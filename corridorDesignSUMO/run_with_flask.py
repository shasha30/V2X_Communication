#!/usr/bin/env python3
# flask_traci_webui.py
# Browser-based controller for SUMO / TraCI monitor + optional SUMO-GUI in browser using Xvfb/x11vnc/noVNC

import os, sys, time, subprocess, threading, json, signal
from flask import Flask, request, jsonify, Response, render_template_string
import math
# (Reuse your existing traci-monitor code imports when starting the monitor thread)
# For brevity, monitor loop implementation is imported from monitor_module.py if available,
# otherwise a simple stub is used. Replace with your real monitor_loop function.

# If you have the monitor loop in a module, import it:
# from my_monitor_module import monitor_loop, stop_monitor_loop, monitor_status, snapshot

# For this file to be self-contained I'll provide a lightweight stubbed monitor that
# simulates vehicle snapshots if you don't have the full monitor available.
monitor_thread = None
monitor_stop_event = threading.Event()
_monitor_lock = threading.Lock()
_monitor_state = {"running": False, "pid": None, "last_error": None}
_snapshot = {"vehicles": [], "rsu_recent": [], "server_time": time.time()}

app = Flask(__name__, static_url_path="/static")

# ----------------- SUMO-GUI via Xvfb/x11vnc helpers -----------------
# These helper functions start/stop an X virtual framebuffer, x11vnc, and SUMO-GUI,
# and expose the VNC through websockify/noVNC. The code below attempts to be robust
# but requires that xvfb-run, x11vnc and noVNC/websockify are installed on the host.

_vnc_procs = {"xvfb": None, "x11vnc": None, "sumo": None, "websockify": None}
_VNC_DISPLAY = 1   # uses :1
_VNC_PORT = 5900 + _VNC_DISPLAY   # vnc port 5901 for :1
_noVNC_port = 6080

def start_vnc_and_sumo(sumocfg, sumo_gui_binary="sumo-gui", geometry="1280x720", xvfb_cmd="Xvfb"):
    """
    Start Xvfb (virtual display :1), start x11vnc on that display, then start sumo-gui on DISPLAY=:1.
    Also start websockify (noVNC) to serve the VNC as a browser page on port _noVNC_port.
    Returns dict of Popen objects.
    """
    global _vnc_procs, _VNC_DISPLAY, _VNC_PORT, _noVNC_port
    if _vnc_procs["xvfb"] is not None:
        return {"ok": False, "reason": "vnc already running"}

    display = f":{_VNC_DISPLAY}"
    # 1) start Xvfb
    xvfb_cmd_line = [xvfb_cmd, display, "-screen", "0", geometry, "-ac"]
    _vnc_procs["xvfb"] = subprocess.Popen(xvfb_cmd_line, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.8)

    # 2) start x11vnc on that display (--forever optional)
    x11vnc_cmd = ["x11vnc", "-display", display, "-nopw", "-forever", "-shared", "-rfbport", str(_VNC_PORT)]
    _vnc_procs["x11vnc"] = subprocess.Popen(x11vnc_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(0.6)

    # 3) start websockify (noVNC) mapping websocket port to VNC port
    # You must have websockify/noVNC installed; typical path is websockify or /usr/share/novnc/utils/novnc_proxy
    websockify_cmd = ["websockify", str(_noVNC_port), f"localhost:{_VNC_PORT}", "--web", "/usr/share/novnc/"] 
    try:
        _vnc_procs["websockify"] = subprocess.Popen(websockify_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except Exception:
        # if websockify not available, continue: user can run noVNC manually outside
        _vnc_procs["websockify"] = None

    time.sleep(0.6)

    # 4) start sumo-gui on that display
    env = os.environ.copy()
    env["DISPLAY"] = display
    sumo_cmd = [sumo_gui_binary, "-c", sumocfg]
    _vnc_procs["sumo"] = subprocess.Popen(sumo_cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(1.0)

    return {"ok": True, "display": display, "vnc_port": _VNC_PORT, "novnc_port": _noVNC_port}

def stop_vnc_and_sumo():
    global _vnc_procs
    for k in ("sumo","websockify","x11vnc","xvfb"):
        p = _vnc_procs.get(k)
        if p:
            try:
                p.terminate()
                p.wait(timeout=2)
            except Exception:
                try: p.kill()
                except Exception: pass
            _vnc_procs[k] = None
    return True

# ----------------- Monitor thread stub (replace with your real monitor loop) -----------------
def _monitor_stub_loop(sumocfg=None, gui=False, additional=None):
    """Simple stub that generates fake vehicle snapshots so UI can show something.
       Replace this with the real monitor_loop that uses traci.
    """
    global _monitor_state, _snapshot
    _monitor_state["running"] = True
    _monitor_state["pid"] = os.getpid()
    try:
        t0 = time.time()
        i = 0
        while not monitor_stop_event.is_set():
            # generate a few fake vehicles (replace with real traci reads)
            _snapshot["vehicles"] = []
            for v in range(1, 21):
                x = 100 + v*5 + 5*math.sin((time.time()+v)/10.0)
                y = 50 + v*0.5
                _snapshot["vehicles"].append({"veh_id": f"veh_{v}", "x": x, "y": y, "speed": abs(10.0 + 2.0*math.sin(time.time()+v)), "heading": 0.0, "timestamp": time.time()})
            _snapshot["rsu_recent"] = _snapshot.get("rsu_recent", [])[-300:]
            _snapshot["server_time"] = time.time()
            time.sleep(0.5)
            i += 1
    finally:
        _monitor_state["running"] = False

def start_monitor_background(sumocfg="corridor.sumocfg", gui=True, additional=None, use_vnc=False):
    """Starts monitor thread; optionally also starts SUMO-GUI under VNC when use_vnc True."""
    global monitor_thread, monitor_stop_event, _vnc_procs, _monitor_state
    if _monitor_state.get("running"):
        return {"ok": False, "reason": "monitor already running"}
    monitor_stop_event.clear()

    if use_vnc:
        res = start_vnc_and_sumo(sumocfg)
        if not res.get("ok"):
            return {"ok": False, "reason": f"vnc start failed: {res.get('reason')}"}
    # start the monitor thread (replace _monitor_stub_loop with your monitor_loop)
    monitor_thread = threading.Thread(target=_monitor_stub_loop, args=(sumocfg, gui, additional), daemon=True)
    monitor_thread.start()
    # small sleep to allow startup
    time.sleep(0.6)
    return {"ok": True, "monitor_running": _monitor_state.get("running", True)}

def stop_monitor_background():
    global monitor_thread, monitor_stop_event
    monitor_stop_event.set()
    stop_vnc_and_sumo()
    # give thread time to stop (monitor thread should check event)
    time.sleep(0.4)
    return {"ok": True}

# ----------------- Flask endpoints -----------------

@app.route("/start", methods=["POST"])
def web_start():
    """
    Start monitor. JSON body:
    { "sumocfg":"corridor.sumocfg", "gui": true, "additional": "corridor.add.xml", "use_vnc": true }
    """
    data = request.get_json(force=False) or {}
    sumocfg = data.get("sumocfg", "corridor.sumocfg")
    gui = bool(data.get("gui", True))
    add = data.get("additional", None)
    use_vnc = bool(data.get("use_vnc", False))
    res = start_monitor_background(sumocfg=sumocfg, gui=gui, additional=add, use_vnc=use_vnc)
    return jsonify(res)

@app.route("/stop", methods=["POST"])
def web_stop():
    res = stop_monitor_background()
    return jsonify(res)

@app.route("/status", methods=["GET"])
def web_status():
    return jsonify({"running": _monitor_state.get("running", False), "last_error": _monitor_state.get("last_error", None)})

@app.route("/snapshot", methods=["GET"])
def web_snapshot():
    # return current in-memory snapshot
    return jsonify(_snapshot)

# ----------------- Browser dashboard page -----------------
DASH_HTML = """
<!doctype html>
<html><head><meta charset="utf-8"><title>SUMO Web UI</title>
<style>
 body{font-family:Arial;margin:0} #top{display:flex;padding:8px;background:#111;color:#fff;align-items:center}
 #top .btn{margin-right:8px;padding:8px 12px;background:#0b6;border-radius:6px;color:#000;cursor:pointer}
 #top .btn.stop{background:#f33;color:#fff} #top .info{margin-left:16px;color:#ddd}
 #container{display:flex;height:calc(100vh - 56px)}
 #left{flex:1;min-width:480px}
 #right{width:480px;border-left:1px solid #ddd;padding:8px;overflow:auto}
 #map{height:calc(100vh - 56px);background:#eee}
 iframe#novnc_frame{width:100%;height:320px;border:1px solid #ccc}
</style>
</head><body>
<div id="top">
  <div id="controls">
    <button class="btn" id="btnStart">Start</button>
    <button class="btn stop" id="btnStop">Stop</button>
    <label style="margin-left:12px"><input type="checkbox" id="chkVNC"> Show SUMO GUI in browser (noVNC)</label>
  </div>
  <div class="info" id="status">Status: stopped</div>
</div>
<div id="container">
  <div id="left">
    <div id="map">Loading map...</div>
  </div>
  <div id="right">
    <h3>SUMO GUI (browser)</h3>
    <div>
      If you enabled VNC (x11vnc + noVNC) the SUMO GUI will appear below.
      <div style="margin-top:8px"><iframe id="novnc_frame" src="" title="noVNC"></iframe></div>
    </div>
    <h3>Recent RSU hits</h3>
    <div id="rsu_list"></div>
  </div>
</div>

<!-- Leaflet -->
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>

<script>
const STATUS = document.getElementById("status");
const btnStart = document.getElementById("btnStart");
const btnStop = document.getElementById("btnStop");
const chkVNC = document.getElementById("chkVNC");
const novnc_frame = document.getElementById("novnc_frame");
const rsu_list = document.getElementById("rsu_list");

btnStart.onclick = async () => {
  STATUS.innerText = "Starting...";
  const use_vnc = chkVNC.checked;
  const body = { "sumocfg":"corridor.sumocfg", "gui": true, "use_vnc": use_vnc, "additional": "corridor.add.xml" };
  const r = await fetch("/start", {method:"POST", headers:{"content-type":"application/json"}, body: JSON.stringify(body)});
  const j = await r.json();
  console.log(j);
  setTimeout(updateStatus, 600);
  if (use_vnc) {
    // novnc served on port 6080 by websockify; adjust if needed
    novnc_frame.src = "/novnc/";  // you may need to serve noVNC files; see instructions
  }
};

btnStop.onclick = async () => {
  STATUS.innerText = "Stopping...";
  const r = await fetch("/stop", {method:"POST"});
  const j = await r.json();
  console.log(j);
  novnc_frame.src = "";
  setTimeout(updateStatus, 400);
};

async function updateStatus(){
  const r = await fetch("/status"); const j = await r.json();
  STATUS.innerText = "Status: " + (j.running ? "running" : "stopped") + (j.last_error ? (" (error: "+j.last_error+")") : "");
}

// --- Leaflet map (CRS.Simple, will display SUMO meters) ---
const map = L.map('map', { crs: L.CRS.Simple }).setView([0,0],0);
const layer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {opacity:0.5}).addTo(map);
let vehMarkers = {};

async function poll(){
  try {
    const r = await fetch("/snapshot"); const j = await r.json();
    const vehs = j.vehicles || [];
    const rsu = j.rsu_recent || [];
    // update vehicles
    const seen = new Set();
    let pts = [];
    for (const v of vehs){
      const lat = v.y, lng = v.x;
      seen.add(v.veh_id);
      pts.push([lat,lng]);
      if (vehMarkers[v.veh_id]) vehMarkers[v.veh_id].setLatLng([lat,lng]);
      else vehMarkers[v.veh_id] = L.circleMarker([lat,lng], {radius:6}).addTo(map).bindPopup(v.veh_id);
    }
    for (const id in vehMarkers) if (!seen.has(id)) { map.removeLayer(vehMarkers[id]); delete vehMarkers[id]; }
    if (pts.length) {
      const bounds = L.latLngBounds(pts);
      map.fitBounds(bounds.pad(0.2));
    }
    // RSU list
    rsu_list.innerHTML = "";
    for (const ritem of rsu.slice(-50).reverse()) {
      const div = document.createElement("div");
      div.innerText = `${ritem.rsu_id} - ${ritem.obj_type}/${ritem.obj_id} - dist=${Number(ritem.distance_m).toFixed(1)}m`;
      rsu_list.appendChild(div);
    }
  } catch(e){ console.warn(e); }
  setTimeout(poll, 1000);
}
updateStatus();
poll();
</script>
</body></html>
"""

@app.route("/dashboard")
def dashboard():
    return render_template_string(DASH_HTML)

# simple noVNC proxy page (if websockify/noVNC is served under /novnc/)
@app.route("/novnc/")
def novnc_index():
    # If you have noVNC installed in /usr/share/novnc, you could reverse-proxy it. This route just
    # redirects browser to the external noVNC server (assumes websockify serves at :6080)
    return ("<html><body>"
            "<p>If noVNC is running, open <a href='http://%s:6080/vnc.html'>noVNC client</a></p>"
            "</body></html>" % request.host.split(":")[0])

# ----------------- run app -----------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    app.run(host=args.host, port=args.port, debug=args.debug)

