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
from collections import deque

# ---------------- Flask base app ----------------
app = Flask(__name__)

# ---------------- Excel logging utilities ----------------
_excel_lock = threading.Lock()
EXCEL_PATH = "server_results.xlsx"
SHEET_NAME = "server_results"
"""
def _append_rows_to_excel(rows, excel_path=EXCEL_PATH, sheet_name=SHEET_NAME):
    if not rows:
        return
    df_new = pd.DataFrame(rows)
    cols = [
        "timestamp_utc",
        "vehicle_id",
        "record_type",            # "ssm" | "alert" | "safe" | "vru_ssm" | "rsu_detection"
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
        # RSU detection fields
        "rsu_id",
        "object_type",
        "object_id",
        "object_x",
        "object_y",
        "object_distance_m",
        "object_speed_mps",
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
"""
# ---------------- State (recent vehicle positions for live map) ----------------
# vid -> {"position": (x,y), "speed": v, "heading": deg, "timestamp": t}
vehicle_states = {}

# keep last N RSU detections in memory as well (for quick JSON snapshot)
_recent_rsu = deque(maxlen=2000)

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
    dx, dy = pos_rel
    ux, uy = unit_vector(dx, dy)
    rvx, rvy = heading_speed_vec
    return -(rvx * ux + rvy * uy)

def compute_ttc(distance, closing_speed):
    if closing_speed <= 0 or distance <= 0:
        return float('inf')
    return distance / closing_speed

def required_deceleration(rel_speed, distance, cushion=0.0):
    d_eff = max(distance - cushion, 1e-3)
    return (rel_speed ** 2) / (2.0 * d_eff)

def time_headway(distance, follower_speed):
    if follower_speed <= 0:
        return float('inf')
    return distance / follower_speed

# ---------------- VRU endpoint ----------------
@app.route('/v2x/check/vru', methods=['POST'])
def vru_check_risk():
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
        return jsonify(resp)

    except Exception as e:
        return jsonify({"error": str(e)}), 400

@app.route('/v2x/check', methods=['POST'])
def vru_check_alias():
    return vru_check_risk()

# ---------------- Vehicle↔Vehicle endpoint ----------------
@app.route('/v2x/check/vehicle', methods=['POST'])
def check_vehicle_risk():
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
        return jsonify({"vehicle_id": vid, "ssm": ssm_list, "alerts": alerts})

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

# ---------------- RSU detections endpoint (from run.py) ----------------
@app.route('/v2x/check/rsu', methods=['POST'])
def rsu_check():
    try:
        d = request.get_json(force=True)
        # Accept the flat dict format produced by your run.py
        rid = d.get("rsu_id")
        obj_type = d.get("obj_type")
        obj_id = d.get("obj_id")
        # store into memory ring for quick JSON snapshot
        d["_ts"] = time.time()
        _recent_rsu.append(d)

        # Log into Excel (record_type=rsu_detection)
        row = {
            "timestamp_utc": time.time(),
            "vehicle_id": None,
            "record_type": "rsu_detection",
            "rsu_id": rid,
            "object_type": obj_type,
            "object_id": obj_id,
            "object_x": d.get("obj_x"),
            "object_y": d.get("obj_y"),
            "object_distance_m": d.get("distance_m"),
            "object_speed_mps": d.get("speed_mps"),
            "raw_payload": json.dumps(d)
        }
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 400

# ---------------- JSON snapshots for Dash ----------------
@app.route('/v2x/snapshot', methods=['GET'])
def snapshot():
    """
    Small JSON for Dash to poll quickly.
    Returns live vehicles + last ~N RSU detections (from memory).
    """
    try:
        vehicles = []
        for vid, v in vehicle_states.items():
            vehicles.append({
                "veh_id": vid,
                "x": v["position"][0],
                "y": v["position"][1],
                "speed": v["speed"],
                "heading": v.get("heading", 0.0),
                "timestamp": v.get("timestamp", 0.0)
            })
        rsu = list(_recent_rsu)
        return jsonify({"vehicles": vehicles, "rsu_recent": rsu, "server_time": time.time()})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# ---------------- Simple static plot (kept) ----------------
@app.route('/v2x/plot', methods=['GET'])
def plot_vehicle_map():
    # kept for compatibility; the Dash map is richer
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_title("V2X Vehicle Map (risk pairs highlighted)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")

    vids = list(vehicle_states.keys())
    ax.grid(True); ax.set_aspect('equal', adjustable='datalim')

    for vid in vids:
        v = vehicle_states[vid]
        (x, y), hdg = v["position"], v.get("heading", 0.0)
        ax.plot(x, y, 'bo', markersize=4)
        ax.text(x + 1, y + 1, vid, fontsize=7)
        ax.arrow(x, y, 3*math.cos(math.radians(hdg)), 3*math.sin(math.radians(hdg)),
                 head_width=0.8, color='blue', length_includes_head=True)

    buf = io.BytesIO()
    plt.tight_layout()
    plt.savefig(buf, format='png', bbox_inches='tight', dpi=120)
    plt.close(fig)
    buf.seek(0)
    return send_file(buf, mimetype='image/png')

@app.route('/download/excel', methods=['GET'])
def download_excel():
    if not os.path.exists(EXCEL_PATH):
        return jsonify({"error": "No Excel yet"}), 404
    return send_file(EXCEL_PATH, as_attachment=True, download_name=EXCEL_PATH)

# ---------------- Dash app (mounted at /dash) ----------------
def _safe_read_excel(n_rows=10000):
    """Read recent rows from Excel safely; return empty DataFrame on failure."""
    if not os.path.exists(EXCEL_PATH):
        return pd.DataFrame()
    with _excel_lock:
        try:
            df = pd.read_excel(EXCEL_PATH, sheet_name=SHEET_NAME, engine="openpyxl")
        except Exception as e:
            print("[WARN] read_excel failed:", e)
            return pd.DataFrame()
    if len(df) > n_rows:
        df = df.tail(n_rows).copy()
    return df

def init_dash(flask_app: Flask):
    from dash import Dash, dcc, html, dash_table, Input, Output  # Dash 2.x
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
            html.Div(id=id_value, style={"fontSize": 24, "fontWeight": "600"})
        ], style={"padding":"10px 14px", "border":"1px solid #ddd", "borderRadius":"10px", "background":"#fafafa"})

    dash_app.layout = html.Div([
        html.H3("V2X Surrogate Safety Dashboard", style={"margin":"10px 0 6px"}),
        html.Div([
            kpi_card("Vehicles online", "kpi_vehicles"),
            kpi_card("Alerts (last 10 min)", "kpi_alerts"),
            kpi_card("RSU detections (last 10 min)", "kpi_rsu"),
        ], style={"display":"grid", "gridTemplateColumns":"repeat(3, 1fr)", "gap":"10px"}),

        html.Div([
            html.Div([
                html.H4("Live Vehicle Map"),
                dcc.Graph(id="live_map", style={"height":"420px"})
            ], style={"flex":"2", "minWidth":"420px"}),
            html.Div([
                html.H4("Recent Alerts"),
                dash_table.DataTable(
                    id="tbl_alerts",
                    columns=[{"name":c, "id":c} for c in ["timestamp_utc","vehicle_id","alert_type","alert_to","risk_score","recommended_action","ttc_s"]],
                    page_size=8,
                    style_table={"height":"420px", "overflowY":"auto"},
                    style_cell={"fontSize":12, "padding":"6px"},
                )
            ], style={"flex":"1", "minWidth":"320px", "paddingLeft":"10px"})
        ], style={"display":"flex", "gap":"10px", "marginTop":"12px"}),

        html.Div([
            html.Div([
                html.H4("TTC Histogram (SSM)"),
                dcc.Graph(id="hist_ttc", style={"height":"300px"})
            ], style={"flex":"1"}),
            html.Div([
                html.H4("RSU Detections (table)"),
                dash_table.DataTable(
                    id="tbl_rsu",
                    columns=[{"name":c, "id":c} for c in ["timestamp_utc","rsu_id","object_type","object_id","object_distance_m","object_speed_mps"]],
                    page_size=8,
                    style_table={"height":"300px", "overflowY":"auto"},
                    style_cell={"fontSize":12, "padding":"6px"},
                )
            ], style={"flex":"1", "paddingLeft":"10px"}),
        ], style={"display":"flex", "gap":"10px", "marginTop":"12px"}),

        dcc.Interval(id="timer", interval=2000, n_intervals=0)  # 2s refresh
    ], style={"fontFamily":"Arial, sans-serif", "padding":"12px 18px"})

    @dash_app.callback(
        Output("kpi_vehicles","children"),
        Output("kpi_alerts","children"),
        Output("kpi_rsu","children"),
        Output("live_map","figure"),
        Output("tbl_alerts","data"),
        Output("hist_ttc","figure"),
        Output("tbl_rsu","data"),
        Input("timer","n_intervals")
    )
    def refresh(_):
        import requests as _rq
        # 1) Live snapshot for vehicles/RSU recent
        try:
            snap = _rq.get("http://127.0.0.1:5000/v2x/snapshot", timeout=0.6).json()
        except Exception:
            snap = {"vehicles": [], "rsu_recent": []}

        vehs = snap.get("vehicles", [])
        rsu_recent = snap.get("rsu_recent", [])
        kpi_veh = len(vehs)
        kpi_rsu = len(rsu_recent)

        # 2) Read Excel (last rows)
        df = _safe_read_excel(n_rows=8000)
        df["timestamp_utc"] = pd.to_datetime(df.get("timestamp_utc", pd.Series(dtype=float)), unit="s", errors="coerce")

        # KPIs for alerts in last 10 minutes
        alerts_last = 0
        alerts_tbl = []
        ttc_hist_fig = {}
        rsu_tbl = []

        if not df.empty:
            t_now = pd.Timestamp.utcnow()
            df_recent = df[df["timestamp_utc"] >= (t_now - pd.Timedelta(minutes=10))]

            # Alerts KPI & table
            df_alerts = df_recent[df_recent["record_type"] == "alert"].copy()
            alerts_last = len(df_alerts)
            alerts_tbl = df_alerts.sort_values("timestamp_utc", ascending=False)[
                ["timestamp_utc","vehicle_id","alert_type","alert_to","risk_score","recommended_action","ttc_s"]
            ].head(20).fillna("").to_dict("records")

            # TTC histogram from SSM rows
            df_ssm = df_recent[(df_recent["record_type"] == "ssm") & (df_recent["ttc_s"].notna())].copy()
            if not df_ssm.empty:
                import plotly.express as px
                ttc_hist_fig = px.histogram(df_ssm, x="ttc_s", nbins=40, opacity=0.85,
                                            labels={"ttc_s": "TTC [s]"},
                                            title="TTC distribution (last 10 min)")
            else:
                import plotly.graph_objects as go
                ttc_hist_fig = go.Figure()
                ttc_hist_fig.update_layout(title="TTC distribution (no SSM rows)")

            # RSU detections table (last 10 min)
            df_rsu = df_recent[df_recent["record_type"] == "rsu_detection"].copy()
            if not df_rsu.empty:
                rsu_tbl = df_rsu.sort_values("timestamp_utc", ascending=False)[
                    ["timestamp_utc","rsu_id","object_type","object_id","object_distance_m","object_speed_mps"]
                ].head(20).fillna("").to_dict("records")

        # 3) Live map from snapshot
        import plotly.graph_objects as go
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
            # draw RSU detections as faint markers near RSU center (optional)
            fig.add_trace(go.Scatter(
                x=[d.get("rsu_x") for d in rsu_recent],
                y=[d.get("rsu_y") for d in rsu_recent],
                mode="markers",
                marker={"symbol":"x", "size":10},
                name="RSUs (recent hits)"
            ))
        fig.update_layout(
            title="Live Map (Vehicles & RSU recent hits)",
            xaxis_title="X [m]", yaxis_title="Y [m]",
            height=420
        )
        # KPI strings
        return str(kpi_veh), str(alerts_last), str(kpi_rsu), fig, alerts_tbl, ttc_hist_fig, rsu_tbl

    return dash_app

# Mount Dash
dash_app = init_dash(app)

# ---------------- Basic HTML page (optional quick preview) ----------------
@app.route('/')
def dashboard():
    return Response("""
    <!doctype html>
    <html><head>
      <title>V2X Dashboard</title>
      <style>body{font-family:Arial;margin:18px} a{font-size:16px}</style>
    </head><body>
      <h2>V2X Surrogate Safety</h2>
      <p><a href="/dash" target="_blank">Open Dash Dashboard</a></p>
      <p><a href="/download/excel">Download server_results.xlsx</a></p>
      <p><img src="/v2x/plot" width="820"/></p>
    </body></html>
    """, mimetype="text/html")

# ---------------- Main ----------------
if __name__ == '__main__':
    # Run locally; change host to "0.0.0.0" for LAN access
    app.run(host='127.0.0.1', port=5000, debug=True)

