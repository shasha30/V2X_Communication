#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Surrogate Safety Server + Dash Dashboard (Excel-free, in-memory)

Endpoints your SUMO TraCI client (run.py) can call:
  - POST /v2x/check/vehicle   : vehicle-vs-vehicle SSMs + alerts
  - POST /v2x/check/vru       : vehicle-vs-pedestrian SSMs + alert
  - POST /v2x/check/rsu       : RSU detections
  - GET  /v2x/snapshot        : compact JSON snapshot for dashboard
  - GET  /dash                : interactive dashboard (Plotly Dash)
"""

from flask import Flask, request, jsonify, Response
import json
import math
import time
from collections import deque
import statistics

# ---------------- Flask app ----------------
app = Flask(__name__)

# ---------------- In-memory state ----------------
# Latest vehicle state: vid -> {position:(x,y), speed, heading, timestamp}
vehicle_states = {}

# Ring buffers for the last N records (RAM only)
BUF_SIZE = 20000
ssm_buf = deque(maxlen=BUF_SIZE)     # vehicle-vehicle SSM rows
vru_buf = deque(maxlen=BUF_SIZE)     # vehicle-vru SSM rows
alert_buf = deque(maxlen=BUF_SIZE)   # alerts generated
rsu_buf = deque(maxlen=BUF_SIZE)     # RSU detections

# --------------- SSM math helpers ---------------
def euclidean_distance(p1, p2):
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)

def unit_vector(vx, vy):
    mag = math.hypot(vx, vy)
    if mag == 0.0:
        return 0.0, 0.0
    return vx / mag, vy / mag

def project_speed_along_line(pos_rel, rel_vel_vec):
    """
    pos_rel = other - ego (dx, dy)
    rel_vel_vec = ego_v - other_v (vx, vy)
    returns positive if closing, negative if separating
    """
    dx, dy = pos_rel
    ux, uy = unit_vector(dx, dy)
    rvx, rvy = rel_vel_vec
    return -(rvx * ux + rvy * uy)

def compute_ttc(distance, closing_speed):
    if closing_speed <= 0 or distance <= 0:
        return float('inf')
    return distance / closing_speed

def required_deceleration(rel_speed, distance, cushion=0.0):
    """
    Constant-deceleration approximation:
      a_req = v_rel^2 / (2 * (distance - cushion))
    """
    d_eff = max(distance - cushion, 1e-3)
    return (rel_speed ** 2) / (2.0 * d_eff)

def time_headway(distance, follower_speed):
    if follower_speed <= 0:
        return float('inf')
    return distance / follower_speed

def safe_percentile(values, p):
    """Return p-th percentile (0..100) or None."""
    vals = [x for x in values if isinstance(x, (int, float))]
    if not vals:
        return None
    vals.sort()
    k = (len(vals) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return vals[int(k)]
    return vals[f] + (k - f) * (vals[c] - vals[f])

# --------------- REST: Vehicle ↔ Vehicle ---------------
@app.route("/v2x/check/vehicle", methods=["POST"])
def check_vehicle_risk():
    """
    Body: {"id": "veh_1", "position":[x,y], "speed": v, "heading": deg}
    Produces SSMs vs all known vehicles + alerts. Stores in memory.
    """
    try:
        data = request.get_json(force=True)
        ts = time.time()

        vid = data["id"]
        pos = tuple(data["position"])
        speed = float(data.get("speed", 0.0))
        heading = float(data.get("heading", 0.0))

        # update ego state
        vehicle_states[vid] = {"position": pos, "speed": speed, "heading": heading, "timestamp": ts}

        ssm_list = []
        alerts = []

        # compute SSM vs others
        for other_id, other in vehicle_states.items():
            if other_id == vid:
                continue

            opos = tuple(other["position"])
            ospeed = float(other.get("speed", 0.0))
            ohead  = float(other.get("heading", 0.0))

            # relative geometry
            distance = euclidean_distance(pos, opos)

            # velocity vectors
            vx,  vy  = speed  * math.cos(math.radians(heading)),  speed  * math.sin(math.radians(heading))
            ovx, ovy = ospeed * math.cos(math.radians(ohead)),    ospeed * math.sin(math.radians(ohead))

            pos_rel = (opos[0] - pos[0], opos[1] - pos[1])
            rel_vx, rel_vy = vx - ovx, vy - ovy

            closing = project_speed_along_line(pos_rel, (rel_vx, rel_vy))
            ttc     = compute_ttc(distance, closing)
            delta_v = abs(math.hypot(rel_vx, rel_vy))
            req_dec = required_deceleration(delta_v, distance)
            thw     = time_headway(distance, speed)

            # PET (very simple proxy): difference of arrival times to the current line
            # If both are moving toward each other (closing>0) and ego has speed>0, estimate:
            if closing > 0 and speed > 0:
                pet = abs((distance / closing) - (distance / max(speed, 1e-6)))
            else:
                pet = float('inf')

            ssm = {
                "other_id": other_id,
                "distance": round(distance, 3),
                "closing_speed": round(closing, 3),
                "delta_v": round(delta_v, 3),
                "ttc": None if ttc == float('inf') else round(ttc, 3),
                "required_deceleration": round(req_dec, 3),
                "time_headway": None if thw == float('inf') else round(thw, 3),
                "pet": None if pet == float('inf') else round(pet, 3),
            }
            ssm_list.append(ssm)

            # store compact SSM in buffer (raw floats, not rounded)
            ssm_buf.append({
                "ts": ts,
                "ego": vid,
                "other": other_id,
                "dist": distance,
                "closing": closing,
                "ttc": None if ttc == float('inf') else ttc,
                "req_dec": req_dec,
                "thw": None if thw == float('inf') else thw,
                "delta_v": delta_v,
                "pet": None if pet == float('inf') else pet,
            })

            # Simple risk model
            risk = 0.0
            if ttc != float('inf'):
                risk += 0.6 if ttc < 1.0 else (0.3 if ttc < 2.5 else 0.0)
            if req_dec > 5.0:
                risk += 0.2
            if delta_v > 5.0:
                risk += 0.2
            if pet is not None and pet != float('inf') and pet < 1.0:
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
                alert_buf.append({
                    "ts": ts, "type": alert["type"], "from": vid, "to": other_id,
                    "risk": risk, "action": "emergency_brake",
                    "ttc": None if ttc == float('inf') else ttc
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
                alert_buf.append({
                    "ts": ts, "type": alert["type"], "from": vid, "to": other_id,
                    "risk": risk, "action": "slow_down",
                    "ttc": None if ttc == float('inf') else ttc
                })

        if not alerts:
            alerts = [{"action": "safe", "timestamp": ts}]

        return jsonify({"vehicle_id": vid, "ssm": ssm_list, "alerts": alerts})

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

# --------------- REST: Vehicle ↔ VRU ---------------
@app.route("/v2x/check/vru", methods=["POST"])
def vru_check_risk():
    """
    Body:
    {
      "vehicle": {"id":"veh_1","position":[x,y],"speed":v,"heading":deg},
      "pedestrian": {"id":"ped_1","position":[x,y],"speed":vp,"heading":deg}
    }
    """
    try:
        payload = request.get_json(force=True)
        ts = time.time()

        v = payload["vehicle"]
        p = payload["pedestrian"]

        vpos = tuple(v["position"]); vs = float(v.get("speed", 0.0)); vh = float(v.get("heading", 0.0))
        ppos = tuple(p["position"]); ps = float(p.get("speed", 0.0)); ph = float(p.get("heading", 0.0))

        dist = euclidean_distance(vpos, ppos)

        vvx, vvy = vs * math.cos(math.radians(vh)), vs * math.sin(math.radians(vh))
        pvx, pvy = ps * math.cos(math.radians(ph)), ps * math.sin(math.radians(ph))

        rel_vx, rel_vy = vvx - pvx, vvy - pvy
        pos_rel = (ppos[0] - vpos[0], ppos[1] - vpos[1])

        closing = project_speed_along_line(pos_rel, (rel_vx, rel_vy))
        ttc     = compute_ttc(dist, closing)
        pet     = abs((dist/ps) - (dist/closing)) if (ps > 0 and closing > 0) else float('inf')
        delta_v = abs(math.hypot(rel_vx, rel_vy))
        req_dec = required_deceleration(delta_v, dist)
        thw     = time_headway(dist, vs)

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
            "timestamp": ts
        }

        vru_buf.append({
            "ts": ts, "veh_id": v.get("id"), "ped_id": p.get("id"),
            "dist": dist, "closing": closing,
            "ttc": None if ttc == float('inf') else ttc,
            "pet": None if pet == float('inf') else pet,
            "req_dec": req_dec,
            "thw": None if thw == float('inf') else thw,
            "risk": risk, "action": action
        })

        return jsonify(resp)

    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route("/v2x/check", methods=["POST"])
def vru_check_alias():
    return vru_check_risk()

# --------------- REST: RSU detections ---------------
@app.route("/v2x/check/rsu", methods=["POST"])
def rsu_check():
    """
    Body (from run.py RSU detections):
    {
      "rsu_id": "...", "rsu_x": ..., "rsu_y": ...,
      "obj_type": "vehicle"|"pedestrian", "obj_id": "...",
      "obj_x": ..., "obj_y": ..., "distance_m": ..., "speed_mps": ...,
      "sim_time": ...
    }
    """
    try:
        d = request.get_json(force=True)
        ts = time.time()
        rsu_buf.append({
            "ts": ts,
            "rsu_id": d.get("rsu_id"),
            "obj_type": d.get("obj_type"),
            "obj_id": d.get("obj_id"),
            "rsu_x": d.get("rsu_x"),
            "rsu_y": d.get("rsu_y"),
            "obj_x": d.get("obj_x"),
            "obj_y": d.get("obj_y"),
            "distance": d.get("distance_m"),
            "speed": d.get("speed_mps"),
        })
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

# --------------- Snapshot for Dash ---------------
@app.route("/v2x/snapshot", methods=["GET"])
def snapshot():
    try:
        now = time.time()

        # vehicles (live)
        vehicles = [{
            "veh_id": vid,
            "x": v["position"][0],
            "y": v["position"][1],
            "speed": v["speed"],
            "heading": v.get("heading", 0.0),
            "timestamp": v.get("timestamp", 0.0)
        } for vid, v in vehicle_states.items()]

        # last 60s RSU detections (for map)
        rsu_recent = [r for r in rsu_buf if (now - r["ts"]) <= 60.0]

        # last 10 min alerts and SSMs (for tables/hists)
        alerts_recent = [a for a in alert_buf if (now - a["ts"]) <= 600.0]
        ssm_recent    = [s for s in ssm_buf   if (now - s["ts"]) <= 600.0]

        return jsonify({
            "vehicles": vehicles,
            "rsu_recent": rsu_recent,
            "alerts_recent": alerts_recent,
            "ssm_recent": ssm_recent,
            "server_time": now
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# --------------- Dash Dashboard ---------------
def init_dash(flask_app: Flask):
    from dash import Dash, dcc, html, dash_table, Input, Output
    import plotly.graph_objects as go
    import plotly.express as px

    dash_app = Dash(
        __name__,
        server=flask_app,
        url_base_pathname="/dash/",
        suppress_callback_exceptions=True
    )

    def kpi_card(title, id_value):
        return html.Div([
            html.Div(title, style={"fontSize": 13, "color": "#666"}),
            html.Div(id=id_value, style={"fontSize": 22, "fontWeight": "600"})
        ], style={"padding":"10px 14px","border":"1px solid #ddd","borderRadius":"10px","background":"#fafafa"})

    dash_app.layout = html.Div([
        html.H3("V2X Surrogate Safety Dashboard (in-memory)", style={"margin":"8px 0 4px"}),

        html.Div([
            kpi_card("Vehicles online", "kpi_vehicles"),
            kpi_card("Alerts (last 10 min)", "kpi_alerts"),
            kpi_card("RSU hits (last 60 s)", "kpi_rsu"),
            kpi_card("Median TTC (last 10 min)", "kpi_ttc"),
            kpi_card("95th pct required decel", "kpi_dec95"),
        ], style={"display":"grid","gridTemplateColumns":"repeat(5, 1fr)","gap":"10px"}),

        html.Div([
            html.Div([
                html.H4("Live Vehicle Map"),
                dcc.Graph(id="live_map", style={"height":"440px"})
            ], style={"flex":"2","minWidth":"440px"}),
            html.Div([
                html.H4("Recent Alerts"),
                dash_table.DataTable(
                    id="tbl_alerts",
                    columns=[{"name":c,"id":c} for c in ["time","type","from","to","risk","action","ttc"]],
                    page_size=10,
                    style_table={"height":"440px","overflowY":"auto"},
                    style_cell={"fontSize":12,"padding":"6px"},
                )
            ], style={"flex":"1","minWidth":"340px","paddingLeft":"10px"})
        ], style={"display":"flex","gap":"10px","marginTop":"10px"}),

        html.Div([
            html.Div([
                html.H4("TTC Histogram (last 10 min)"),
                dcc.Graph(id="hist_ttc", style={"height":"280px"})
            ], style={"flex":"1"}),
            html.Div([
                html.H4("PET Histogram (last 10 min)"),
                dcc.Graph(id="hist_pet", style={"height":"280px"})
            ], style={"flex":"1"}),
            html.Div([
                html.H4("Required Deceleration Histogram (last 10 min)"),
                dcc.Graph(id="hist_dec", style={"height":"280px"})
            ], style={"flex":"1"}),
        ], style={"display":"flex","gap":"10px","marginTop":"10px"}),

        html.Div([
            html.Div([
                html.H4("RSU Detections (last 10 min)"),
                dash_table.DataTable(
                    id="tbl_rsu",
                    columns=[{"name":c,"id":c} for c in ["time","rsu_id","obj_type","obj_id","distance","speed"]],
                    page_size=10,
                    style_table={"height":"300px","overflowY":"auto"},
                    style_cell={"fontSize":12,"padding":"6px"},
                )
            ], style={"flex":"1"})
        ], style={"display":"flex","gap":"10px","marginTop":"10px"}),

        dcc.Interval(id="timer", interval=2000, n_intervals=0)
    ], style={"fontFamily":"Arial, sans-serif","padding":"12px 18px"})

    @dash_app.callback(
        Output("kpi_vehicles","children"),
        Output("kpi_alerts","children"),
        Output("kpi_rsu","children"),
        Output("kpi_ttc","children"),
        Output("kpi_dec95","children"),
        Output("live_map","figure"),
        Output("tbl_alerts","data"),
        Output("hist_ttc","figure"),
        Output("hist_pet","figure"),
        Output("hist_dec","figure"),
        Output("tbl_rsu","data"),
        Input("timer","n_intervals")
    )
    def refresh(_):
        import requests as _rq
        try:
            snap = _rq.get("http://127.0.0.1:5000/v2x/snapshot", timeout=0.7).json()
        except Exception:
            snap = {"vehicles": [], "rsu_recent": [], "alerts_recent": [], "ssm_recent": []}

        vehs         = snap.get("vehicles", [])
        rsu_recent   = snap.get("rsu_recent", [])
        alerts_recent= snap.get("alerts_recent", [])
        ssm_recent   = snap.get("ssm_recent", [])

        # KPIs
        kpi_veh = str(len(vehs))
        kpi_alerts = str(len(alerts_recent))
        kpi_rsu = str(len(rsu_recent))

        ttc_vals = [s["ttc"] for s in ssm_recent if isinstance(s.get("ttc"), (int, float))]
        dec_vals = [s["req_dec"] for s in ssm_recent if isinstance(s.get("req_dec"), (int, float))]
        pet_vals = [s["pet"] for s in ssm_recent if isinstance(s.get("pet"), (int, float))]

        kpi_ttc = f"{statistics.median(ttc_vals):.1f} s" if ttc_vals else "—"
        dec95 = safe_percentile(dec_vals, 95)
        kpi_dec = f"{dec95:.2f} m/s²" if dec95 is not None else "—"

        # Live map (vehicles + RSU hits)
        fig = go.Figure()
        if vehs:
            fig.add_trace(go.Scatter(
                x=[v["x"] for v in vehs], y=[v["y"] for v in vehs],
                mode="markers+text",
                text=[v["veh_id"] for v in vehs],
                textposition="top center",
                marker={"size":8},
                name="Vehicles"
            ))
        if rsu_recent:
            fig.add_trace(go.Scatter(
                x=[d["rsu_x"] for d in rsu_recent if d.get("rsu_x") is not None],
                y=[d["rsu_y"] for d in rsu_recent if d.get("rsu_y") is not None],
                mode="markers",
                marker={"symbol":"x","size":10},
                name="RSUs (recent hits)"
            ))
        fig.update_layout(title="Live Map (Vehicles & RSU recent hits)",
                          xaxis_title="X [m]", yaxis_title="Y [m]", height=440)

        # Alerts table (last 10 min already filtered in /snapshot)
        alerts_tbl = [{
            "time": time.strftime("%H:%M:%S", time.localtime(a["ts"])),
            "type": a["type"],
            "from": a["from"],
            "to": a["to"],
            "risk": f"{a['risk']:.2f}",
            "action": a["action"],
            "ttc": "∞" if a["ttc"] is None else f"{a['ttc']:.2f}"
        } for a in sorted(alerts_recent, key=lambda z: z["ts"], reverse=True)[:50]]

        # Histograms
        if ttc_vals:
            hist_ttc = px.histogram({"ttc": ttc_vals}, x="ttc", nbins=40, opacity=0.85,
                                    labels={"ttc":"TTC [s]"}, title="TTC distribution (last 10 min)")
        else:
            hist_ttc = go.Figure(); hist_ttc.update_layout(title="TTC distribution (no data)")

        if pet_vals:
            hist_pet = px.histogram({"pet": pet_vals}, x="pet", nbins=40, opacity=0.85,
                                    labels={"pet":"PET [s]"}, title="PET distribution (last 10 min)")
        else:
            hist_pet = go.Figure(); hist_pet.update_layout(title="PET distribution (no data)")

        if dec_vals:
            hist_dec = px.histogram({"a_req": dec_vals}, x="a_req", nbins=40, opacity=0.85,
                                    labels={"a_req":"Required deceleration [m/s²]"},
                                    title="Required Deceleration (last 10 min)")
        else:
            hist_dec = go.Figure(); hist_dec.update_layout(title="Required Deceleration (no data)")

        # RSU table: use last 10 min from rsu_buf (or rsu_recent is last 60s only)
        now = time.time()
        rsu_last10 = [r for r in rsu_buf if (now - r["ts"]) <= 600.0]
        rsu_tbl = [{
            "time": time.strftime("%H:%M:%S", time.localtime(r["ts"])),
            "rsu_id": r.get("rsu_id"),
            "obj_type": r.get("obj_type"),
            "obj_id": r.get("obj_id"),
            "distance": f"{(r.get('distance') or 0):.2f}",
            "speed": f"{(r.get('speed') or 0):.2f}",
        } for r in sorted(rsu_last10, key=lambda z: z["ts"], reverse=True)[:50]]

        return kpi_veh, kpi_alerts, kpi_rsu, kpi_ttc, kpi_dec, fig, alerts_tbl, hist_ttc, hist_pet, hist_dec, rsu_tbl

    return dash_app

dash_app = init_dash(app)

# --------------- Root (simple link) ---------------
@app.route("/")
def index():
    return Response("""
    <!doctype html>
    <html><head>
      <title>V2X Dashboard</title>
      <style>body{font-family:Arial;margin:18px} a{font-size:16px}</style>
    </head><body>
      <h2>V2X Surrogate Safety (in-memory)</h2>
      <p><a href="/dash" target="_blank">Open Dash Dashboard</a></p>
    </body></html>
    """, mimetype="text/html")

# --------------- Main ---------------
if __name__ == "__main__":
    # Change host to "0.0.0.0" for LAN access
    app.run(host="10.45.0.1", port=6000, debug=True)
