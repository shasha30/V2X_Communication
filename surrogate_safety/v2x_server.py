# v2x_server.py (simplified)
from flask import Flask, request, jsonify,send_file
import math
import time
import matplotlib.pyplot as plt
import io


app = Flask(__name__)


# Global dictionary to store latest vehicle data
vehicle_states = {}

def euclidean_distance(pos1, pos2):
    dx = pos1[0] - pos2[0]
    dy = pos1[1] - pos2[1]
    return math.sqrt(dx**2 + dy**2)

def will_collide(pos1, pos2, speed1, speed2, threshold=10):
    distance = euclidean_distance(pos1, pos2)
    rel_speed = abs(speed1 - speed2)
    if distance < threshold and rel_speed > 3:
        return True, distance, rel_speed
    return False, distance, rel_speed

def predict_position(pos, speed, heading_deg, t=1.0):
    heading_rad = math.radians(heading_deg)
    dx = speed * t * math.cos(heading_rad)
    dy = speed * t * math.sin(heading_rad)
    return [pos[0] + dx, pos[1] + dy]
    

@app.route('/v2x/check/vru', methods=['POST'])
def vru_check_risk():
    v = request.json["vehicle"]
    print("vehicle",v)
    p = request.json["pedestrian"]
    print("pedestrian",p)
    dist = math.sqrt((v["position"][0] - v["position"][0])**2 + (v["position"][1] - p["position"][1])**2)
    print(" distance b/w vehicle and pedestrian is ",dist)
    if dist < 20 and v["speed"] > 3:
        return jsonify({"action": "slow_down"})
    return jsonify({"action": "keep"})


@app.route('/v2x/check/vehicle', methods=['POST'])
def check_vehicle_risk():
    try:
        data = request.get_json(force=True)

        # Extract incoming vehicle data
        vid = data["id"]
        pos = data["position"]
        speed = data["speed"]
        heading = data.get("heading", 0.0)  # Optional

        # Store current state
        vehicle_states[vid] = {
            "position": pos,
            "speed": speed,
            "heading": heading,
            "timestamp": time.time()
        }

        alerts = []

        # Check for potential collisions with other vehicles
        for other_id, other in vehicle_states.items():
            if other_id == vid:
                continue

            # Simple collision check
            collision, distance, rel_speed = will_collide(pos, other["position"], speed, other["speed"])

            # Predict future positions for advanced logic (optional)
            future_pos1 = predict_position(pos, speed, heading, t=1)
            future_pos2 = predict_position(other["position"], other["speed"], other.get("heading", 0.0), t=1)
            future_distance = euclidean_distance(future_pos1, future_pos2)

            if collision or future_distance < 5:
                alerts.append({
                    "type": "collision_warning",
                    "from": vid,
                    "to": other_id,
                    "distance_now": round(distance, 2),
                    "distance_predicted": round(future_distance, 2),
                    "relative_speed": round(rel_speed, 2),
                    "recommended_action": "slow_down" if rel_speed > 3 else "brake",
                    "severity": "high" if distance < 5 else "medium",
                    "timestamp": time.time()
                })
                print(alerts)
        return jsonify({
            "vehicle_id": vid,
            "alerts": alerts if alerts else [{"action": "safe", "timestamp": time.time()}]
        })

    except Exception as e:
        print("Error processing request:", e)
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
 
app.run(port=5000)

