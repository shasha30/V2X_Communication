#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run.py
Monitor Vehicles, Pedestrians, RSUs/POIs, Lanes/Edges, and Detectors from SUMO via TraCI.
Sends data to Flask SSM server for analysis.

Outputs:
  - vehicles_log.csv
  - pedestrians_log.csv
  - rsus_log.csv
  - rsu_detections.csv
  - lanes_log.csv
  - edges_log.csv
  - detectors_log.csv
"""
import os
import sys
import csv
import time
import math
import argparse
import requests

# --- SUMO / TraCI bootstrap ---
def _add_sumo_tools():
    if "SUMO_HOME" in os.environ:
        tools = os.path.join(os.environ["SUMO_HOME"], "tools")
        if tools not in sys.path:
            sys.path.append(tools)
    else:
        print("[WARN] SUMO_HOME not set. Set SUMO_HOME to your SUMO install path if import fails.")

_add_sumo_tools()

try:
    import traci
    from sumolib import checkBinary
except Exception as e:
    print("[FATAL] Could not import TraCI / sumolib. Ensure SUMO is installed and SUMO_HOME is set.\n", e)
    sys.exit(1)


# -------- CSV helpers --------
class CsvWriter:
    def __init__(self, path, headers):
        self.path = path
        self.headers = headers
        self._fh = open(self.path, "w", newline="")
        self._wr = csv.DictWriter(self._fh, fieldnames=self.headers)
        self._wr.writeheader()
        self.flush()

    def write(self, row: dict):
        for h in self.headers:
            row.setdefault(h, "")
        self._wr.writerow(row)

    def flush(self):
        self._fh.flush()
        os.fsync(self._fh.fileno())

    def close(self):
        try:
            self._fh.close()
        except:
            pass


# -------- RSU abstraction --------
def list_rsus():
    rsus = []
    if hasattr(traci, "rsu"):
        try:
            for rid in traci.rsu.getIDList():
                x, y = traci.rsu.getPosition(rid)
                rsus.append({"id": rid, "x": x, "y": y, "type": "native_rsu"})
        except Exception:
            pass

    try:
        for pid in traci.poi.getIDList():
            ptype = ""
            try:
                ptype = traci.poi.getType(pid)
            except Exception:
                pass
            if str(ptype).upper() == "RSU":
                x, y = traci.poi.getPosition(pid)
                rng = ""
                try:
                    rng = traci.poi.getParameter(pid, "range_m")
                except Exception:
                    pass
                rsus.append({"id": pid, "x": x, "y": y, "type": "poi_rsu", "range_m": rng})
    except Exception:
        pass
    return rsus


# -------- RSU detection --------
def _float_range(rsu_dict_entry, default=200.0):
    try:
        return float(rsu_dict_entry.get("range_m", default))
    except Exception:
        return default


def rsu_detect(rsu_list, veh_ids, ped_ids):
    detections = []
    for rsu in rsu_list:
        rx, ry = rsu["x"], rsu["y"]
        rng = _float_range(rsu)
        rid = rsu["id"]

        # vehicles
        for vid in veh_ids:
            try:
                vx, vy = traci.vehicle.getPosition(vid)
                vspd = traci.vehicle.getSpeed(vid)
                d = math.hypot(vx - rx, vy - ry)
                if d <= rng:
                    detections.append({
                        "rsu_id": rid, "rsu_x": rx, "rsu_y": ry,
                        "obj_type": "vehicle", "obj_id": vid,
                        "obj_x": vx, "obj_y": vy,
                        "distance_m": d, "speed_mps": vspd
                    })
            except Exception:
                pass

        # pedestrians
        for pid in ped_ids:
            try:
                px, py = traci.person.getPosition(pid)
                ps = traci.person.getSpeed(pid)
                d = math.hypot(px - rx, py - ry)
                if d <= rng:
                    detections.append({
                        "rsu_id": rid, "rsu_x": rx, "rsu_y": ry,
                        "obj_type": "pedestrian", "obj_id": pid,
                        "obj_x": px, "obj_y": py,
                        "distance_m": d, "speed_mps": ps
                    })
            except Exception:
                pass
    return detections


# -------- Detector helpers --------
def poll_detectors():
    rows = []
    if hasattr(traci, "inductionloop"):
        for did in traci.inductionloop.getIDList():
            try:
                vehs = traci.inductionloop.getLastStepVehicleIDs(did)
                flow = traci.inductionloop.getLastStepVehicleNumber(did)
                mean_speed = traci.inductionloop.getLastStepMeanSpeed(did)
                rows.append({
                    "detector_id": did, "type": "e1",
                    "veh_count_last_step": flow,
                    "veh_ids": " ".join(vehs),
                    "mean_speed": round(mean_speed, 3)
                })
            except Exception:
                pass
    return rows


# -------- Helpers for allowed-lane fixes --------
def set_lane_change_mode_for_vehicle(veh_id):
    """
    Set a permissive lane change mode so vehicles can change lanes to escape bad lanes.
    1621 is commonly used to allow most lane-change behavior (useful for mixed traffic).
    """
    try:
        traci.vehicle.setLaneChangeMode(veh_id, 1621)
    except Exception:
        # Some SUMO versions may throw for short-lived vehicles; ignore.
        pass


def _lane_allows_vclass(lane_id, vclass):
    """
    Returns True if lane allows the vclass (e.g., 'passenger', 'motorcycle').
    If lane.getAllowed fails or returns empty, treat as allowed for safety.
    """
    try:
        allowed = traci.lane.getAllowed(lane_id)
        # if allowed is empty or contains 'all', treat as permissive
        if not allowed:
            return True
        # allowed items may be bytes/strings; normalize
        allowed_strs = [str(a).strip() for a in allowed]
        if "all" in allowed_strs or "ALL" in allowed_strs:
            return True
        # If vclass appears, allow
        if vclass in allowed_strs:
            return True
        # Some networks use vType ids in allowed; also check
        return False
    except Exception:
        # If the API is not available, be permissive
        return True


def find_allowed_lane_on_edge(vtype_id, edge_id):
    """
    Search lanes on the given edge for a lane that allows the vehicle's vclass (vtype_id).
    Returns target laneIndex (int) if found else None.
    We assume lanes named edgeid_0, edgeid_1, ...
    """
    try:
        # obtain vehicle class from vType
        vclass = None
        try:
            vclass = traci.vehicle.getVehicleClass(vtype_id)
        except Exception:
            # fallback: many vType -> vClass not directly accessible; assume mapping by vtype name
            # crude mapping:
            if vtype_id.startswith("car"):
                vclass = "passenger"
            elif vtype_id.startswith("mtw") or vtype_id.startswith("auto"):
                vclass = "motorcycle"
            elif vtype_id.startswith("truck"):
                vclass = "truck"
            elif vtype_id.startswith("bus"):
                vclass = "bus"
            elif vtype_id.startswith("bike"):
                vclass = "bicycle"
            else:
                vclass = "passenger"
    except Exception:
        vclass = "passenger"

    # try get lane count for edge
    try:
        lane_count = traci.edge.getLaneNumber(edge_id)
    except Exception:
        # fallback: try scanning lane indices until one exists
        lane_count = 0
        while True:
            lid = f"{edge_id}_{lane_count}"
            try:
                traci.lane.getLength(lid)
                lane_count += 1
                if lane_count > 20:
                    break
            except Exception:
                break
        # if none found, return None
        if lane_count == 0:
            return None

    for lane_idx in range(lane_count):
        lane_id = f"{edge_id}_{lane_idx}"
        if _lane_allows_vclass(lane_id, vclass):
            return lane_idx
    return None


# -------- Main --------
def main(args):
    binary = checkBinary("sumo-gui" if args.gui else "sumo")
    sumo_cmd = [binary, "-c", args.sumocfg]
    if args.step_length is not None:
        sumo_cmd += ["--step-length", str(args.step_length)]
    if args.additional:
        sumo_cmd += ["--additional-files", args.additional]

    print("[INFO] Starting SUMO:", " ".join(sumo_cmd))
    traci.start(sumo_cmd)
    print("[INFO] TraCI connected.")

    veh_csv = CsvWriter("vehicles_log.csv", [
        "sim_time", "veh_id", "type", "edge", "lane", "lane_pos",
        "x", "y", "speed", "accel", "angle", "route_id", "leader_id", "gap_to_leader"
    ])
    ped_csv = CsvWriter("pedestrians_log.csv", [
        "sim_time", "person_id", "edge", "lane", "x", "y", "speed", "stage"
    ])
    rsu_csv = CsvWriter("rsus_log.csv", ["sim_time", "rsu_id", "x", "y", "type", "range_m"])
    rsu_det_csv = CsvWriter("rsu_detections.csv", [
        "sim_time", "rsu_id", "rsu_x", "rsu_y", "obj_type", "obj_id", "obj_x", "obj_y", "distance_m", "speed_mps"
    ])
    lanes_csv = CsvWriter("lanes_log.csv", [
        "sim_time", "lane_id", "edge_id", "allowed_vclasses", "max_speed",
        "mean_speed_last_step", "veh_count_last_step"
    ])
    edges_csv = CsvWriter("edges_log.csv", [
        "sim_time", "edge_id", "mean_speed_last_step", "traveltime_last_step", "veh_count_last_step"
    ])
    det_csv = CsvWriter("detectors_log.csv", [
        "sim_time", "detector_id", "type", "veh_count_last_step", "mean_speed", "veh_ids"
    ])

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            t = traci.simulation.getTime()

            # --- Vehicles ---
            vids = traci.vehicle.getIDList()
            for vid in vids:
                try:
                    x, y = traci.vehicle.getPosition(vid)
                    speed = traci.vehicle.getSpeed(vid)
                    angle = traci.vehicle.getAngle(vid)
                    lane_id = traci.vehicle.getLaneID(vid)
                    edge_id = traci.vehicle.getRoadID(vid)
                    lane_pos = traci.vehicle.getLanePosition(vid)
                    vtype = traci.vehicle.getTypeID(vid)
                    route_id = ""
                    try:
                        route_id = traci.vehicle.getRouteID(vid)
                    except Exception:
                        pass
                    accel = ""
                    try:
                        accel = traci.vehicle.getAcceleration(vid)
                    except Exception:
                        pass

                    # Ensure permissive lane-change mode to allow escaping bad lanes
                    set_lane_change_mode_for_vehicle(vid)

                    # get leader info if available
                    leader_id, gap = (None, None)
                    try:
                        leader = traci.vehicle.getLeader(vid, 1000)
                        if leader:
                            leader_id, gap = leader
                    except Exception:
                        leader_id, gap = (None, None)

                    # check if current lane allows this vehicle type; if not, try to move to allowed lane on same edge
                    try:
                        if lane_id and edge_id:
                            allowed_ok = _lane_allows_vclass(lane_id, vtype)
                            if not allowed_ok:
                                target_lane_idx = find_allowed_lane_on_edge(vtype, edge_id)
                                if target_lane_idx is not None:
                                    try:
                                        # perform lane change towards target lane index (duration 2.0s)
                                        traci.vehicle.changeLane(vid, int(target_lane_idx), 2.0)
                                        print(f"[INFO] Requested lane change for {vid} to lane {edge_id}_{target_lane_idx} (was {lane_id})")
                                    except Exception as e:
                                        print(f"[WARN] changeLane failed for {vid}: {e}")
                                else:
                                    print(f"[WARN] No allowed lane found on edge {edge_id} for vtype {vtype} (vehicle {vid})")
                    except Exception as e:
                        print(f"[WARN] lane-allowance check failed for {vid}: {e}")

                    # --- Send vehicle data to Flask ---
                    payload = {"id": vid, "position": [x, y], "speed": speed, "heading": angle}
                    try:
                        r = requests.post("http://10.45.0.1:6000/v2x/check/vehicle", json=payload, timeout=5.0)
                        if r.ok:
                            resp = r.json()
                            print(f"[SSM] {vid} alerts={resp.get('alerts')}")
                        else:
                            print(f"[WARN] Flask responded {r.status_code}")
                    except Exception as e:
                        print(f"[ERROR] V2X vehicle endpoint error for {vid}: {e}")

                    veh_csv.write({
                        "sim_time": t, "veh_id": vid, "type": vtype,
                        "edge": edge_id, "lane": lane_id, "lane_pos": round(lane_pos, 2),
                        "x": round(x, 2), "y": round(y, 2), "speed": round(speed, 3),
                        "accel": accel if accel == "" else round(accel, 3),
                        "angle": round(angle, 2), "route_id": route_id,
                        "leader_id": leader_id if leader_id else "",
                        "gap_to_leader": "" if gap is None else round(gap, 3)
                    })
                except Exception as e:
                    print(f"[WARN] vehicle read failed for {vid}: {e}")

            # --- Pedestrians ---
            pids = traci.person.getIDList()
            for pid in pids:
                try:
                    x, y = traci.person.getPosition(pid)
                    speed = traci.person.getSpeed(pid)
                    edge = traci.person.getRoadID(pid)
                    lane = traci.person.getLaneID(pid)
                    stage = ""
                    try:
                        stage = traci.person.getStage(pid).type
                    except Exception:
                        pass
                    ped_csv.write({
                        "sim_time": t, "person_id": pid, "edge": edge, "lane": lane,
                        "x": round(x, 2), "y": round(y, 2), "speed": round(speed, 3),
                        "stage": stage
                    })
                except Exception as e:
                    print(f"[WARN] pedestrian read failed for {pid}: {e}")

            # --- RSUs ---
            rsus = list_rsus()
            for r in rsus:
                rsu_csv.write({
                    "sim_time": t, "rsu_id": r.get("id"),
                    "x": round(r.get("x", 0.0), 2), "y": round(r.get("y", 0.0), 2),
                    "type": r.get("type", ""), "range_m": r.get("range_m", "")
                })

            # RSU detections + send to Flask
            detections = rsu_detect(rsus, vids, pids)
            for d in detections:
                d["sim_time"] = t
                rsu_det_csv.write(d)
                try:
                    r = requests.post("http://10.45.0.1:6000/v2x/check/rsu", json=d, timeout=5.0)
                    if not r.ok:
                        print(f"[WARN] Flask RSU endpoint responded {r.status_code}")
                except Exception as e:
                    print(f"[ERROR] V2X RSU endpoint error: {e}")

            # --- Detectors ---
            for row in poll_detectors():
                row["sim_time"] = t
                det_csv.write(row)

            # --- Flush CSVs ---
            veh_csv.flush(); ped_csv.flush(); rsu_csv.flush()
            rsu_det_csv.flush(); lanes_csv.flush(); edges_csv.flush(); det_csv.flush()

            if int(t) % 5 == 0:
                print(f"[{t:6.1f}s] vehicles={len(vids)} peds={len(pids)} rsus={len(rsus)}")

        print("[INFO] Simulation ended.")

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")
    finally:
        try:
            traci.close()
        except:
            pass
        for w in (veh_csv, ped_csv, rsu_csv, rsu_det_csv, lanes_csv, edges_csv, det_csv):
            try:
                w.close()
            except:
                pass
        print("[INFO] Closed TraCI and CSVs. Bye!")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Monitor and log SUMO state via TraCI.")
    ap.add_argument("--sumocfg", required=True, help="Path to .sumocfg")
    ap.add_argument("--gui", action="store_true", help="Use sumo-gui instead of sumo")
    ap.add_argument("--step-length", type=float, default=None, help="Override step-length (s)")
    ap.add_argument("--additional", type=str, default=None, help="Additional files")
    args = ap.parse_args()
    main(args)

