import traci
import requests
import csv
import math
import json
sumo_cmd = ["sumo-gui", "-c", "osm.sumocfg"]
traci.start(sumo_cmd)
print("[INFO] SUMO GUI started.")

def get_entity_color(entity_id):
    if entity_id.startswith("veh"):
        return (255, 0, 0, 255)       # Red for cars
    elif entity_id.startswith("bike"):
        return (0, 0, 255, 255)       # Blue for bikes
    elif entity_id.startswith("cycle"):
        return (0, 255, 0, 255)       # Green for bicycles
    elif entity_id.startswith("ped"):
        return (255, 255, 0, 255)     # Yellow for pedestrians
    else:
        return (200, 200, 200, 255)   # Default gray
"""
# 🔄 Persistent CSV writer
log_file = open("alerts.csv", "w", newline="")
writer = csv.writer(log_file)
writer.writerow(["time", "vehicle", "pedestrian", "action", "distance"])
"""
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    veh_ids = traci.vehicle.getIDList()
    ped_ids = traci.person.getIDList()

    #print(f"[STEP {traci.simulation.getTime()}] Active Vehicles: {veh_ids} | Pedestrians: {ped_ids}")

    for veh_id in veh_ids:
        vpos = traci.vehicle.getPosition(veh_id)
        vspeed = traci.vehicle.getSpeed(veh_id)
        traci.vehicle.setColor(veh_id, get_entity_color(veh_id))
        print(" vehicle position is ",vpos)
        print(" vehicle speed is ",vspeed)
        payload={"id": veh_id, "position": vpos, "speed": vspeed}
        r = requests.post("http://localhost:5000/v2x/check/vehicle", json=payload)
        print(r.json())
        for pid in ped_ids:
            ppos = traci.person.getPosition(pid)
            traci.person.setColor(pid, get_entity_color(pid))

            #distance = math.sqrt((vpos[0] - ppos[0])**2 + (vpos[1] - ppos[1])**2)

            payload = {
                "vehicle": {"id": veh_id, "position": vpos, "speed": vspeed},
                "pedestrian": {"id": pid, "position": ppos}
            }
            print(payload)
            try:
                r = requests.post("http://localhost:5000/v2x/check", json=payload)
                action = r.json().get("action", "keep")
                print(f"[DATA] {veh_id} @ {vpos}, {pid} @ {ppos}, Distance: {distance:.2f}, Action: {action}")
                writer.writerow([traci.simulation.getTime(), veh_id, pid, action, f"{distance:.2f}"])

                if action == "slow_down":
                    traci.vehicle.setSpeed(veh_id, 3.0)
            except Exception as e:
                print(f"[ERROR] V2X server error: {e}")

traci.close()
#log_file.close()
print("[INFO] Simulation completed.")

