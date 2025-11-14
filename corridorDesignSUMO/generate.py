#!/usr/bin/env python3
"""
generate_routes_fixed.py

Generates a SUMO .rou.xml with:
 - vType definitions (using SUMO-known guiShape names)
 - route elements with explicit edge lists (so route ids are valid)
 - explicit <vehicle> entries (mixed traffic)

Usage:
  python3 generate_routes_fixed.py -n 1000 -d 600 -o corridor.rou.xml
"""
import argparse, random, math, xml.etree.ElementTree as ET, sys
from typing import Dict, List

# --- default routes -> EDGE sequences
# IMPORTANT: Adjust these edge lists to match your network edge ids exactly.
ROUTE_EDGE_MAP = {
    "east_main":  "A0_A1 A1_A2 A2_A3 A3_A4 A4_A5",
    "west_main":  "A5_A4 A4_A3 A3_A2 A2_A1 A1_A0",
    "south2east": "S2S_A2 A2_A3 A3_A4",
    "north2west": "S2N_A2 A2_A1 A1_A0",
    "south4north":"S4S_A4 A4_S4N",
    "north4east": "S4N_A4 A4_A5"
}

DEFAULT_PROPORTIONS = {"car":0.55, "mtw":0.20, "auto":0.10, "truck":0.08, "bus":0.04, "bike":0.03}

# Use SUMO-known guiShape values (avoid "threewheel")
VTYPE_DEFS = {
    "car":  dict(accel="1.5", decel="3.0", sigma="0.6", length="4.5", width="1.7", maxSpeed="15.0", vClass="passenger", guiShape="passenger"),
    "mtw":  dict(accel="2.0", decel="4.0", sigma="0.9", length="2.2", width="0.9",  maxSpeed="18.0", vClass="motorcycle", guiShape="motorcycle"),
    "auto": dict(accel="1.6", decel="3.0", sigma="0.8", length="3.0", width="1.2",  maxSpeed="13.0", vClass="motorcycle", guiShape="motorcycle"),
    "truck":dict(accel="0.8", decel="2.0", sigma="0.5", length="8.0", width="2.5",  maxSpeed="11.0", vClass="truck", guiShape="truck"),
    "bus":  dict(accel="1.0", decel="2.0", sigma="0.6", length="12.0",width="2.6",  maxSpeed="12.0", vClass="bus", guiShape="bus"),
    "bike": dict(accel="1.0", decel="2.0", sigma="0.9", length="1.8", width="0.6",  maxSpeed="6.0",  vClass="bicycle", guiShape="bicycle"),
}

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("-n","--num-vehicles", type=int, required=True)
    p.add_argument("-d","--duration", type=float, default=600.0)
    p.add_argument("-o","--out", default="corridor.rou.xml")
    p.add_argument("--mode", choices=["uniform","poisson"], default="uniform")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--routes", type=str, default=",".join(ROUTE_EDGE_MAP.keys()))
    p.add_argument("--departLane", choices=["random","best","first"], default="random")
    p.add_argument("--departSpeed", choices=["max","random"], default="max")
    p.add_argument("--pedestrians", type=int, default=50)
    return p.parse_args()

def choose_vehicle_types(count:int, proportions:Dict[str,float], rng:random.Random)->List[str]:
    types = list(proportions.keys())
    probs = [proportions[t] for t in types]
    counts = {t: int(math.floor(p*count)) for t,p in zip(types, probs)}
    assigned = sum(counts.values()); rem = count - assigned
    for _ in range(rem):
        idx = rng.choices(range(len(types)), weights=probs, k=1)[0]
        counts[types[idx]] += 1
    out=[]
    for t in types:
        out.extend([t]*counts[t])
    rng.shuffle(out); return out

def gen_depart_times(count:int, duration:float, mode:str, rng:random.Random)->List[float]:
    if count<=0: return []
    times=[]
    if mode=="uniform":
        for i in range(count):
            t = (i + rng.random()*0.9999) * (duration / max(1,count))
            times.append(round(t,3))
    else:
        rate = count / max(1e-6, duration); t=0.0
        while len(times)<count:
            inter = rng.expovariate(rate)
            t += inter
            if t>duration:
                while len(times)<count:
                    times.append(round(rng.random()*duration,3))
                break
            times.append(round(t,3))
        times.sort()
    return times

def build_tree(num, duration, out, mode, seed, departLane, departSpeed, routes_list, pedestrians):
    rng = random.Random(seed)
    proportions = DEFAULT_PROPORTIONS.copy()
    vtypes_list = choose_vehicle_types(num, proportions, rng)
    depart_times = gen_depart_times(num, duration, mode, rng)
    # root
    root = ET.Element("routes")
    # vTypes
    for vid, attrs in VTYPE_DEFS.items():
        v = ET.SubElement(root, "vType", id=vid)
        for k,val in attrs.items(): v.set(k,str(val))
    # route elements with explicit edges
    for rid in routes_list:
        edges = ROUTE_EDGE_MAP.get(rid, "")
        if edges=="":
            print(f"[WARN] route '{rid}' has no mapping in ROUTE_EDGE_MAP; SUMO will error if used.")
        ET.SubElement(root, "route", id=rid, edges=edges)
    # vehicles
    for i in range(num):
        vid = f"veh_{i+1}"
        vtype = vtypes_list[i]
        depart = depart_times[i]
        route = rng.choice(routes_list)
        attrib = {"id":vid, "type":vtype, "route":route, "depart":f"{depart:.3f}", "departLane":departLane, "departSpeed":departSpeed}
        ET.SubElement(root, "vehicle", attrib=attrib)
    # pedestrians
    if pedestrians>0:
        pf = ET.SubElement(root, "personFlow", id="ped_main", type="ped", begin="0", end=str(int(duration)), number=str(pedestrians), departPos="random")
        # Example walk edges: adjust if your edges differ
        ET.SubElement(pf, "walk", edges=ROUTE_EDGE_MAP.get("east_main","A0_A1 A1_A2 A2_A3 A3_A4 A4_A5"))
    # pretty-print & save
    def indent(elem, level=0):
        i="\n"+level*"  "
        if len(elem):
            if not elem.text or not elem.text.strip(): elem.text = i+"  "
            for e in elem: indent(e, level+1)
            if not elem.tail or not elem.tail.strip(): elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()): elem.tail = i
    indent(root)
    tree = ET.ElementTree(root)
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"[OK] wrote {out}")

def main():
    args = parse_args()
    routes_list = [r.strip() for r in args.routes.split(",") if r.strip()]
    build_tree(args.num_vehicles, args.duration, args.out, args.mode, args.seed, args.departLane, args.departSpeed, routes_list, args.pedestrians)

if __name__=="__main__":
    main()

