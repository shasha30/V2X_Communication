#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, request, jsonify, send_file, Response
import math
import time
import threading
import pandas as pd
import json
import os
import io

# plotting
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt

app = Flask(__name__)

# ---------------- Excel logging utilities ----------------
_excel_lock = threading.Lock()
EXCEL_PATH = "server_results.xlsx"
SHEET_NAME = "server_results"

def _append_rows_to_excel(rows, excel_path=EXCEL_PATH, sheet_name=SHEET_NAME):
    """
    rows: list[dict] with columns below. Safely appends by reading, concat, rewrite.
    """
    if not rows:
        return

    df_new = pd.DataFrame(rows)

    cols = [
        "timestamp_utc",
        "vehicle_id",
        "record_type",            # "ssm" | "alert" | "safe" | "vru_ssm"
        # ssm fields
        "other_id",
        "distance_m",
        "closing_speed_mps",
        "delta_v_mps",
        "ttc_s",
        "required_deceleration_mps2",
        "time_headway_s",
        # alert fields
        "alert_type",
        "alert_from",
        "alert_to",
        "risk_score",
        "recommended_action",
        # VRU-only extras
        "pet_s",
        # raw
        "raw_payload"
    ]
    for c in cols:
        if c not in df_new.columns:
            df_new[c] = pd.NA
    df_new = df_new[cols]

    with _excel_lock:
        if os.path.exists(excel_path):
            try:
                existing = pd.read_excel(excel_path, sheet_name=sheet_name, engine="openpyxl")
                combined = pd.concat([existing, df_new], ignore_index=True)
            except Exception:
                combined = df_new
            with pd.ExcelWriter(excel_path, engine="openpyxl", mode="w") as writer:
                combined.to_excel(writer, sheet_name=sheet_name, index=False)
        else:
            with pd.ExcelWriter(excel_path, engine="openpyxl", mode="w") as writer:
                df_new.to_excel(writer, sheet_name=sheet_name, index=False)

# ---------------- State ----------------
# Latest known vehicles (populated by /v2x/check/vehicle calls from SUMO/TraCI)
vehicle_states = {}  # vid -> {"position": (x,y), "speed": v, "heading": deg, "timestamp": t}

# ---------------- Helper math ----------------
def euclidean_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

def unit_vector(vx, vy):
    mag = math.hypot(vx, vy)
    if mag == 0.0:
        return 0.0, 0.0
    return vx / mag, vy / mag

def project_speed_along_line(pos_rel, heading_speed_vec):
    """
    pos_rel = other - ego (dx, dy)
    heading_speed_vec = ego_v - other_v (vx, vy)
    returns +ve when closing, -ve when separating
    """
    dx, dy = pos_rel
    ux, uy = unit_vector(dx, dy)
    rvx, rvy = heading_speed_vec
    return -(rvx * ux + rvy * uy)

def compute_ttc(distance, closing_speed):
    """ seconds until collision if closing_speed>0 (else inf) """
    if closing_speed <= 0 or distance <= 0:
        return float('inf')
    return distance / closing_speed

def required_deceleration(rel_speed, distance, cushion=0.0):
    """ a = v^2 / (2d) with a small cushion on distance """
    d_eff = max(distance - cushion, 1e-3)
    return (rel_speed ** 2) / (2.0 * d_eff)

def time_headway(distance, follower_speed):
    if follower_speed <= 0:
        return float('inf')
    return distance / follower_speed

# Quick pair-risk used for the plot (very simple, uses TTC threshold)
def will_collide(p1, p2, v1, v2, h1=0.0, h2=0.0, ttc_thresh=2.0):
    """
    Return (is_risky, distance, rel_speed_closing) using line-of-approach projection.
    """
    d = euclidean_distance(p1, p2)
    v1x, v1y = v1 * math.cos(math.radians(h1)), v1 * math.sin(math.radians(h1))
    v2x, v2y = v2 * math.cos(math.radians(h2)), v2 * math.sin(math.radians(h2))
    pos_rel = (p2[0] - p1[0], p2[1] - p1[1])
    rel_vx = v1x - v2x
    rel_vy = v1y - v2y
    closing = project_speed_along_line(pos_rel, (rel_vx, rel_vy))
    ttc = compute_ttc(d, closing)
    return (ttc != float('inf') and ttc < ttc_thresh), d, closing

# ---------------- VRU endpoint (vehicle ↔ pedestrian) ----------------
@app.route('/v2x/check/vru', methods=['POST'])
def vru_check_risk():
    """
    JSON:
    {
      "vehicle": {"id":"veh_1","position":[x,y],"speed":v,"heading":deg},
      "pedestrian": {"id":"ped_1","position":[x,y],"speed":vp,"heading":deg}
    }
    """
    try:
        payload = request.get_json(force=True)
        v = payload["vehicle"]
        p = payload["pedestrian"]

        vpos = tuple(v["position"])
        ppos = tuple(p["position"])
        vs = float(v.get("speed", 0.0))
        ps = float(p.get("speed", 0.0))
        vh = float(v.get("heading", 0.0))
        ph = float(p.get("heading", 0.0))

        dist = euclidean_distance(vpos, ppos)

        vvx, vvy = vs * math.cos(math.radians(vh)), vs * math.sin(math.radians(vh))
        pvx, pvy = ps * math.cos(math.radians(ph)), ps * math.sin(math.radians(ph))

        rel_vx = vvx - pvx
        rel_vy = vvy - pvy
        pos_rel = (ppos[0] - vpos[0], ppos[1] - vpos[1])

        closing = project_speed_along_line(pos_rel, (rel_vx, rel_vy))
        ttc = compute_ttc(dist, closing)
        pet = abs((dist/ps) - (dist/closing)) if (ps > 0 and closing > 0) else float('inf')
        delta_v = abs(math.hypot(rel_vx, rel_vy))
        req_dec = required_deceleration(delta_v, dist)
        thw = time_headway(dist, vs)

        # risk scoring
        risk = 0.0
        if ttc != float('inf'):
            risk += 0.6 if ttc < 1.0 else (0.3 if ttc < 2.5 else 0.0)
        if pet < 1.0:
            risk += 0.3
        if req_dec > 5.0:
            risk += 0.2
        risk = min(1.0, risk)

        if (ttc < 1.0) or (pet < 0.5) or (req_dec > 6.0):
            action, severity = "emergency_brake", "high"
        elif (ttc < 2.5) or (pet < 1.5) or (req_dec > 3.0):
            action, severity = "slow_down", "medium"
        else:
            action, severity = "keep", "low"

        resp = {
            "vehicle_id": v.get("id"),
            "pedestrian_id": p.get("id"),
            "distance": round(dist, 3),
            "closing_speed": round(closing, 3),
            "delta_v": round(delta_v, 3),
            "ttc": None if ttc == float('inf') else round(ttc, 3),
            "pet": None if pet == float('inf') else round(pet, 3),
            "required_deceleration": round(req_dec, 3),
            "time_headway": None if thw == float('inf') else round(thw, 3),
            "risk_score": round(risk, 3),
            "recommended_action": action,
            "severity": severity,
            "timestamp": time.time()
        }
"""
        # log one row
        try:
            _append_rows_to_excel([{
                "timestamp_utc": time.time(),
                "vehicle_id": v.get("id"),
                "record_type": "vru_ssm",
                "other_id": p.get("id"),
                "distance_m": round(dist, 3),
                "closing_speed_mps": round(closing, 3),
                "delta_v_mps": round(delta_v, 3),
                "ttc_s": None if ttc == float('inf') else round(ttc, 3),
                "required_deceleration_mps2": round(req_dec, 3),
                "time_headway_s": None if thw == float('inf') else round(thw, 3),
                "pet_s": None if pet == float('inf') else round(pet, 3),
                "raw_payload": json.dumps(resp)
            }])
        except Exception as e:
            print("[WARN] Excel append failed (VRU):", e)
"""
        return jsonify(resp)

    except Exception as e:
        return jsonify({"error": str(e)}), 400

# Backward-compat alias: older client posted to /v2x/check for ped
@app.route('/v2x/check', methods=['POST'])
def vru_check_alias():
    return vru_check_risk()

# ---------------- Vehicle↔Vehicle endpoint ----------------
@app.route('/v2x/check/vehicle', methods=['POST'])
def check_vehicle_risk():
    """
    JSON: { "id": "veh_1", "position":[x,y], "speed": v, "heading": deg }
    Returns SSMs vs all other known vehicles + alerts. Appends all rows to Excel.
    """
    try:
        data = request.get_json(force=True)
        vid = data["id"]
        pos = tuple(data["position"])
        speed = float(data.get("speed", 0.0))
        heading = float(data.get("heading", 0.0))

        vehicle_states[vid] = {
            "position": pos,
            "speed": speed,
            "heading": heading,
            "timestamp": time.time()
        }

        ssm_list, alerts = [], []
        excel_rows = []

        for other_id, other in vehicle_states.items():
            if other_id == vid:
                continue

            opos = tuple(other["position"])
            ospeed = float(other.get("speed", 0.0))
            ohead = float(other.get("heading", 0.0))

            distance = euclidean_distance(pos, opos)

            vx, vy = speed * math.cos(math.radians(heading)), speed * math.sin(math.radians(heading))
            ovx, ovy = ospeed * math.cos(math.radians(ohead)), ospeed * math.sin(math.radians(ohead))

            pos_rel = (opos[0] - pos[0], opos[1] - pos[1])
            rel_vx, rel_vy = vx - ovx, vy - ovy

            closing = project_speed_along_line(pos_rel, (rel_vx, rel_vy))
            ttc = compute_ttc(distance, closing)
            delta_v = abs(math.hypot(rel_vx, rel_vy))
            req_dec = required_deceleration(delta_v, distance)
            thw = time_headway(distance, speed)

            ssm = {
                "other_id": other_id,
                "distance": round(distance, 3),
                "closing_speed": round(closing, 3),
                "delta_v": round(delta_v, 3),
                "ttc": None if ttc == float('inf') else round(ttc, 3),
                "required_deceleration": round(req_dec, 3),
                "time_headway": None if thw == float('inf') else round(thw, 3),
            }
            ssm_list.append(ssm)

            excel_rows.append({
                "timestamp_utc": time.time(),
                "vehicle_id": vid,
                "record_type": "ssm",
                "other_id": other_id,
                "distance_m": round(distance, 3),
                "closing_speed_mps": round(closing, 3),
                "delta_v_mps": round(delta_v, 3),
                "ttc_s": None if ttc == float('inf') else round(ttc, 3),
                "required_deceleration_mps2": round(req_dec, 3),
                "time_headway_s": None if thw == float('inf') else round(thw, 3),
                "raw_payload": json.dumps({"ego": vid, "other": other_id, "ssm": ssm})
            })

            # risk score & alerts
            risk = 0.0
            if ttc != float('inf'):
                risk += 0.6 if ttc < 1.0 else (0.3 if ttc < 2.5 else 0.0)
            if req_dec > 5.0:
                risk += 0.2
            if delta_v > 5.0:
                risk += 0.2
            risk = min(1.0, risk)

            if risk >= 0.8:
                alert = {
                    "type": "collision_imminent",
                    "from": vid, "to": other_id,
                    "risk_score": round(risk, 3),
                    "recommended_action": "emergency_brake",
                    "ttc": None if ttc == float('inf') else round(ttc, 3)
                }
                alerts.append(alert)
                excel_rows.append({
                    "timestamp_utc": time.time(),
                    "vehicle_id": vid,
                    "record_type": "alert",
                    "alert_type": alert["type"],
                    "alert_from": alert["from"],
                    "alert_to": alert["to"],
                    "risk_score": alert["risk_score"],
                    "recommended_action": alert["recommended_action"],
                    "alert_ttc_s": alert["ttc"],
                    "raw_payload": json.dumps({"ego": vid, "alert": alert})
                })
            elif risk >= 0.4:
                alert = {
                    "type": "collision_warning",
                    "from": vid, "to": other_id,
                    "risk_score": round(risk, 3),
                    "recommended_action": "slow_down",
                    "ttc": None if ttc == float('inf') else round(ttc, 3)
                }
                alerts.append(alert)
                excel_rows.append({
                    "timestamp_utc": time.time(),
                    "vehicle_id": vid,
                    "record_type": "alert",
                    "alert_type": alert["type"],
                    "alert_from": alert["from"],
                    "alert_to": alert["to"],
                    "risk_score": alert["risk_score"],
                    "recommended_action": alert["recommended_action"],
                    "alert_ttc_s": alert["ttc"],
                    "raw_payload": json.dumps({"ego": vid, "alert": alert})
                })

        # if no alerts produced, write a "safe" sentinel
        if not alerts:
            safe = {"action": "safe", "timestamp": time.time()}
            alerts = [safe]
            excel_rows.append({
                "timestamp_utc": time.time(),
                "vehicle_id": vid,
                "record_type": "alert",
                "alert_type": safe.get("action"),
                "alert_from": vid,
                "alert_to": None,
                "risk_score": None,
                "recommended_action": None,
                "alert_ttc_s": safe.get("timestamp"),
                "raw_payload": json.dumps({"ego": vid, "alerts": [safe]})
            })
"""
        try:
            _append_rows_to_excel(excel_rows, EXCEL_PATH, SHEET_NAME)
        except Exception as e:
            print("[WARN] Excel append failed (vehicle):", e)
"""
        return jsonify({"vehicle_id": vid, "ssm": ssm_list, "alerts": alerts})

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

#rsu detection 
@app.route('/v2x/rsu/detections', methods=['POST'])
def rsu_detections():
    """
    Accepts a single detection or a list of detections.

    Single detection JSON shape:
    {
      "rsu_id": "RSU_A2",
      "rsu": {"x": 600.0, "y": 10.0},
      "object": {"type":"vehicle","id":"veh_1","x":..., "y":..., "distance_m":..., "speed_mps":...},
      "sim_time": 12.3
    }

    Or: { "detections": [ ...same objects... ] }
    """
    try:
        data = request.get_json(force=True)

        # normalize to list
        if "detections" in data and isinstance(data["detections"], list):
            dets = data["detections"]
        else:
            dets = [data]

        rows = []
        for d in dets:
            rid = d.get("rsu_id")
            rpos = d.get("rsu", {}) or {}
            obj = d.get("object", {}) or {}
            ts = time.time()

            # minimal validation
            if not (rid and obj.get("id")):
                # skip bad records but do not fail entire batch
                continue

            rows.append({
                "timestamp_utc": ts,
                "vehicle_id": None,              # not a vehicle-ego record
                "record_type": "rsu_detection",
                "rsu_id": rid,
                "object_type": obj.get("type"),
                "object_id": obj.get("id"),
                "object_x": obj.get("x"),
                "object_y": obj.get("y"),
                "object_distance_m": obj.get("distance_m"),
                "object_speed_mps": obj.get("speed_mps"),
                # keep raw in case we evolve the schema later
                "raw_payload": json.dumps(d)
            })
"""
        if rows:
            try:
                _append_rows_to_excel(rows, EXCEL_PATH, SHEET_NAME)
            except Exception as e:
                print("[WARN] Excel append failed (RSU):", e)
"""
        return jsonify({"ok": True, "accepted": len(rows)})

    except Exception as e:
        return jsonify({"error": str(e)}), 400
        
        
# ---------------- Live plot & download ----------------
@app.route('/v2x/plot', methods=['GET'])
def plot_vehicle_map():
    # build a static snapshot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_title("V2X Vehicle Map (risk pairs highlighted)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

    vids = list(vehicle_states.keys())
    risky_pairs = set()
    alert_msgs = []

    # scatter all vehicles
    for vid in vids:
        v = vehicle_states[vid]
        (x, y), spd, hdg = v["position"], v["speed"], v.get("heading", 0.0)
        ax.plot(x, y, 'bo', markersize=4)
        ax.text(x + 1, y + 1, vid, fontsize=7)
        ax.arrow(x, y, 3*math.cos(math.radians(hdg)), 3*math.sin(math.radians(hdg)),
                 head_width=0.8, color='blue', length_includes_head=True)

    # pairwise simple risk
    for i in range(len(vids)):
        for j in range(i+1, len(vids)):
            v1 = vehicle_states[vids[i]]
            v2 = vehicle_states[vids[j]]
            risky, dist, rel = will_collide(v1["position"], v2["position"], v1["speed"], v2["speed"],
                                            v1.get("heading", 0.0), v2.get("heading", 0.0))
            if risky:
                risky_pairs.add((vids[i], vids[j]))
                alert_msgs.append(f"{vids[i]} ↔ {vids[j]}  d={dist:.1f}m  close={rel:.1f} m/s")

    for a, b in risky_pairs:
        x1, y1 = vehicle_states[a]["position"]
        x2, y2 = vehicle_states[b]["position"]
        ax.plot([x1, x2], [y1, y2], 'r--', linewidth=2)

    ax.grid(True); ax.set_aspect('equal', adjustable='datalim')
    ax.text(0.01, 0.99, f"Vehicles: {len(vids)}\nAlerts: {len(risky_pairs)}",
            transform=ax.transAxes, va='top', fontsize=10,
            bbox=dict(boxstyle="round", fc="w", ec="0.5"))

    if alert_msgs:
        ax.text(1.02, 0.97, "Pairs < 2s TTC:", transform=ax.transAxes, va='top', fontsize=9)
        for i, m in enumerate(alert_msgs[:10]):
            ax.text(1.02, 0.93 - 0.05*i, m, transform=ax.transAxes, fontsize=8)

    buf = io.BytesIO()
    plt.tight_layout()
    plt.savefig(buf, format='png', bbox_inches='tight', dpi=120)
    plt.close(fig)
    buf.seek(0)
    return send_file(buf, mimetype='image/png')

@app.route('/')
def dashboard():
    return Response("""
    <!doctype html>
    <html><head>
      <title>V2X Live Dashboard</title>
      <style>body{font-family:Arial;margin:18px} img{border:1px solid #444}</style>
    </head><body>
      <h2>V2X Vehicle Collision Risk Dashboard</h2>
      <p>Auto-refreshes every 1s</p>
      <img id="map" src="/v2x/plot" width="900"/>
      <script>
        setInterval(()=>{const i=document.getElementById('map');i.src='/v2x/plot?ts='+Date.now();},1000);
      </script>
    </body></html>
    """, mimetype="text/html")

@app.route('/download/excel', methods=['GET'])
def download_excel():
    if not os.path.exists(EXCEL_PATH):
        return jsonify({"error": "No Excel yet"}), 404
    return send_file(EXCEL_PATH, as_attachment=True, download_name=EXCEL_PATH)

# ---------------- Main ----------------
if __name__ == '__main__':
    # Run locally; change host to "0.0.0.0" to expose on LAN
    app.run(host='10.45.0.1', port=5000, debug=True)
