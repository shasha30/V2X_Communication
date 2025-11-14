from flask import Flask, request, jsonify, send_file
import math
import time

app = Flask(__name__)

# Global dictionary to store latest vehicle data
vehicle_states = {}

# ---------- Helper SSM functions ----------

def euclidean_distance(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.hypot(dx, dy)

def unit_vector(vx, vy):
    mag = math.hypot(vx, vy)
    if mag == 0:
        return 0.0, 0.0
    return vx / mag, vy / mag

def project_speed_along_line(pos_rel, heading_speed_vec):
    """
    Project relative velocity onto line-of-approach (vector from ego->other)
    pos_rel: (dx, dy) = other - ego
    heading_speed_vec: (vx, vy) relative velocity (ego_v - other_v)
    Returns relative speed along approach (positive if closing)
    """
    dx, dy = pos_rel
    ux, uy = unit_vector(dx, dy)
    rvx, rvy = heading_speed_vec
    return -(rvx * ux + rvy * uy)  # negative sign so positive when closing

def compute_ttc(distance, closing_speed):
    """Approximate Time-To-Collision (seconds). If closing_speed <= 0 => inf"""
    if closing_speed <= 0 or distance <= 0:
        return float('inf')
    return distance / closing_speed

def required_deceleration(rel_speed, distance, cushion=0.0):
    """
    Required deceleration (m/s^2) to avoid collision assuming constant deceleration:
    a = (v^2) / (2 * d)
    apply small cushion in distance if needed.
    """
    d_eff = max(distance - cushion, 1e-3)
    return (rel_speed ** 2) / (2.0 * d_eff)

def time_headway(distance, follower_speed):
    if follower_speed <= 0:
        return float('inf')
    return distance / follower_speed

# ---------- End helper functions ----------

@app.route('/v2x/check/vru', methods=['POST'])
def vru_check_risk():
    """
    Expected JSON:
    {
      "vehicle": {"id": "veh_1", "position":[x,y], "speed": v, "heading": deg},
      "pedestrian": {"id":"ped_1", "position":[x,y], "speed": v_p, "heading": deg_p (walking direction)}
    }
    """
    try:
        payload = request.get_json(force=True)
        v = payload["vehicle"]
        p = payload["pedestrian"]

        # Basic values
        vpos = tuple(v["position"])
        ppos = tuple(p["position"])
        vspeed = float(v.get("speed", 0.0))
        pspeed = float(p.get("speed", 0.0))
        vhdg = float(v.get("heading", 0.0))   # degrees vehicle heading
        phd = float(p.get("heading", 0.0))    # degrees pedestrian heading (optional)

        # relative geometry
        dist = euclidean_distance(vpos, ppos)

        # approximate velocities as vectors (vx,vy)
        vvx = vspeed * math.cos(math.radians(vhdg))
        vvy = vspeed * math.sin(math.radians(vhdg))
        pvx = pspeed * math.cos(math.radians(phd))
        pvy = pspeed * math.sin(math.radians(phd))

        # relative speed vector (vehicle towards pedestrian)
        rel_vx = vvx - pvx
        rel_vy = vvy - pvy
        pos_rel = (ppos[0] - vpos[0], ppos[1] - vpos[1])  # vector from vehicle -> pedestrian

        # closing speed along line of approach (positive if closing)
        closing_speed = project_speed_along_line(pos_rel, (rel_vx, rel_vy))

        # TTC (vehicle to pedestrian) and PET approximation
        ttc = compute_ttc(dist, closing_speed)

        # PET approx as difference between pedestrian arrival time at conflict point and vehicle arrival time.
        # Approximate conflict point as current midpoint projected along each's heading:
        # (Very simplistic) Time for pedestrian to reach current vehicle's path projection:
        # For robustness, if headings not provided, fallback to simple estimate:
        if pspeed > 0:
            t_ped_arrive = dist / pspeed
        else:
            t_ped_arrive = float('inf')

        if closing_speed > 0:
            t_vehicle_arrive = dist / closing_speed
        else:
            t_vehicle_arrive = float('inf')

        pet = abs(t_vehicle_arrive - t_ped_arrive)

        # relative speed (ΔV) and required deceleration to avoid collision
        delta_v = abs(math.hypot(rel_vx, rel_vy))
        req_dec = required_deceleration(delta_v, dist)

        # THW from vehicle perspective (time headway)
        thw = time_headway(dist, vspeed)

        # simple risk scoring (tunable)
        # Higher risk if TTC small, PET small, req_dec large
        risk_score = 0.0
        if ttc != float('inf'):
            if ttc < 1.0:
                risk_score += 0.6
            elif ttc < 2.5:
                risk_score += 0.3
        if pet < 1.0:
            risk_score += 0.3
        if req_dec > 5.0:  # m/s^2, aggressive threshold
            risk_score += 0.2

        # clamp risk to [0,1]
        risk_score = min(1.0, risk_score)

        # action decision
        if ttc < 1.0 or pet < 0.5 or req_dec > 6.0:
            action = "emergency_brake"
            severity = "high"
        elif ttc < 2.5 or pet < 1.5 or req_dec > 3.0:
            action = "slow_down"
            severity = "medium"
        else:
            action = "keep"
            severity = "low"

        response = {
            "vehicle_id": v.get("id"),
            "pedestrian_id": p.get("id"),
            "distance": round(dist, 3),
            "closing_speed": round(closing_speed, 3),
            "delta_v": round(delta_v, 3),
            "ttc": None if ttc == float('inf') else round(ttc, 3),
            "pet": None if (pet == float('inf') or pet != pet) else round(pet, 3),
            "required_deceleration": round(req_dec, 3),
            "time_headway": None if thw == float('inf') else round(thw, 3),
            "risk_score": round(risk_score, 3),
            "recommended_action": action,
            "severity": severity,
            "timestamp": time.time()
        }
        return jsonify(response)

    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route('/v2x/check/vehicle', methods=['POST'])
def check_vehicle_risk():
    """
    Expected JSON:
      { "id": "veh_1", "position": [x,y], "speed": v, "heading": deg }
    Responds with SSMs computed vs all other known vehicles.
    """
    try:
        data = request.get_json(force=True)

        # Extract incoming vehicle data
        vid = data["id"]
        pos = tuple(data["position"])
        speed = float(data.get("speed", 0.0))
        heading = float(data.get("heading", 0.0))  # degrees

        # Store current state
        vehicle_states[vid] = {
            "position": pos,
            "speed": speed,
            "heading": heading,
            "timestamp": time.time()
        }

        ssm_list = []
        alerts = []

        # iterate through other vehicles and compute SSMs
        for other_id, other in vehicle_states.items():
            if other_id == vid:
                continue

            other_pos = tuple(other["position"])
            other_speed = float(other.get("speed", 0.0))
            other_heading = float(other.get("heading", 0.0))

            # distance
            distance = euclidean_distance(pos, other_pos)

            # velocity vectors
            vx = speed * math.cos(math.radians(heading))
            vy = speed * math.sin(math.radians(heading))
            ovx = other_speed * math.cos(math.radians(other_heading))
            ovy = other_speed * math.sin(math.radians(other_heading))

            # relative vector from ego -> other and relative velocity (ego - other)
            pos_rel = (other_pos[0] - pos[0], other_pos[1] - pos[1])
            rel_vx = vx - ovx
            rel_vy = vy - ovy

            # closing speed along line of approach
            closing_speed = project_speed_along_line(pos_rel, (rel_vx, rel_vy))

            # TTC
            ttc = compute_ttc(distance, closing_speed)

            # ΔV magnitude
            delta_v = abs(math.hypot(rel_vx, rel_vy))

            # required deceleration for ego to avoid collision (approx)
            req_dec = required_deceleration(delta_v, distance)

            # time headway (distance / follower speed assuming ego is follower)
            thw = time_headway(distance, speed)

            # simple SSM record
            ssm = {
                "other_id": other_id,
                "distance": round(distance, 3),
                "closing_speed": round(closing_speed, 3),
                "delta_v": round(delta_v, 3),
                "ttc": None if ttc == float('inf') else round(ttc, 3),
                "required_deceleration": round(req_dec, 3),
                "time_headway": None if thw == float('inf') else round(thw, 3),
            }
            ssm_list.append(ssm)

            # Simple alert logic
            risk_score = 0.0
            if ttc != float('inf'):
                if ttc < 1.0:
                    risk_score += 0.6
                elif ttc < 2.5:
                    risk_score += 0.3
            if req_dec > 5.0:
                risk_score += 0.2
            if delta_v > 5.0:
                risk_score += 0.2
            risk_score = min(1.0, risk_score)

            if risk_score >= 0.8:
                alerts.append({
                    "type": "collision_imminent",
                    "from": vid,
                    "to": other_id,
                    "risk_score": round(risk_score, 3),
                    "recommended_action": "emergency_brake",
                    "ttc": None if ttc == float('inf') else round(ttc, 3)
                })
            elif risk_score >= 0.4:
                alerts.append({
                    "type": "collision_warning",
                    "from": vid,
                    "to": other_id,
                    "risk_score": round(risk_score, 3),
                    "recommended_action": "slow_down",
                    "ttc": None if ttc == float('inf') else round(ttc, 3)
                })

        if not alerts:
            alerts = [{"action": "safe", "timestamp": time.time()}]

        return jsonify({
            "vehicle_id": vid,
            "ssm": ssm_list,
            "alerts": alerts
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500
        
         
@app.route('/v2x/plot', methods=['GET'])
def plot_vehicle_map():
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_title("V2X Vehicle Map with Collision Risk", fontsize=14)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")

    risky_pairs = set()
    alert_messages = []

    # Draw each vehicle
    for vid1, v1 in vehicle_states.items():
        x, y = v1["position"]
        heading = v1["heading"]
        ax.plot(x, y, 'bo')  # blue dot
        ax.text(x + 1, y + 1, vid1, fontsize=8)

        # Heading arrow
        arrow_dx = math.cos(math.radians(heading)) * 3
        arrow_dy = math.sin(math.radians(heading)) * 3
        ax.arrow(x, y, arrow_dx, arrow_dy, head_width=1, color='blue')

        # Compare with others for collision
        for vid2, v2 in vehicle_states.items():
            if vid1 >= vid2:
                continue
            coll, dist, rel_speed = will_collide(v1["position"], v2["position"], v1["speed"], v2["speed"])
            if coll:
                risky_pairs.add((vid1, vid2))
                # Store alert as text
                alert_messages.append(f"⚠ {vid1} ↔ {vid2} | {dist:.1f}m @ Δv={rel_speed:.1f} m/s")

    # Red dashed lines for risky pairs
    for vid1, vid2 in risky_pairs:
        x1, y1 = vehicle_states[vid1]["position"]
        x2, y2 = vehicle_states[vid2]["position"]
        ax.plot([x1, x2], [y1, y2], 'r--', linewidth=2)

    # Show stats on the plot
    stats_text = (
        f"Vehicles: {len(vehicle_states)}\n"
        f"Active Alerts: {len(risky_pairs)}"
    )
    ax.text(0.01, 0.99, stats_text, transform=ax.transAxes,
            fontsize=12, verticalalignment='top', bbox=dict(boxstyle="round", fc="lightgray", ec="black"))

    # Display alert messages on side
    if alert_messages:
        ax.text(1.02, 0.95, "Live Alerts:", transform=ax.transAxes, fontsize=10, fontweight='bold')
        for i, msg in enumerate(alert_messages[:10]):  # Limit to 10 alerts for space
            ax.text(1.02, 0.92 - i * 0.05, msg, transform=ax.transAxes, fontsize=9)

    ax.set_aspect('equal')
    ax.grid(True)
    plt.tight_layout()

    # Return image
    buf = io.BytesIO()
    plt.savefig(buf, format='png', bbox_inches='tight')
    plt.close(fig)
    buf.seek(0)
    return send_file(buf, mimetype='image/png')

@app.route('/')
def dashboard():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>V2X Live Dashboard</title>
        <style>
            body { font-family: Arial; text-align: center; margin-top: 20px; }
            h1 { color: #444; }
            #info { margin-bottom: 10px; }
            img { border: 2px solid #444; }
        </style>
    </head>
    <body>
        <h1>V2X Vehicle Collision Risk Dashboard</h1>
        <div id="info">Auto-refreshes every 1 second</div>
        <img id="map" src="/v2x/plot" width="800" height="600" />
        
        <script>
            setInterval(() => {
                const img = document.getElementById('map');
                const timestamp = new Date().getTime();
                img.src = '/v2x/plot?' + timestamp; // avoid browser cache
            }, 1000);
        </script>
    </body>
    </html>
    '''
 

if __name__ == '__main__':
    # run on all interfaces for remote testing; change host/port as needed
    app.run(host='0.0.0.0', port=5000, debug=True)

