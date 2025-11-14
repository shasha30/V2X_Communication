#!/usr/bin/env python3
"""
Generate routes.rou.xml for SUMO with 50 IDM cars using a custom car logo.
Each vehicle departs at 1-second intervals.
"""

import xml.etree.ElementTree as ET

N = 50                  # number of vehicles
depart_gap = 1.0        # seconds between departures
route_edges = "A_B B_C" # adjust if your network has different edges
output_file = "routes.rou.xml"

routes = ET.Element("routes")

# Define IDM vehicle type with custom logo
vtype = ET.SubElement(routes, "vType", {
    "id": "idm_car",
    "vClass": "passenger",
    "accel": "1.0",
    "decel": "4.0",
    "sigma": "0.5",
    "tau": "1.2",
    "length": "4.5",
    "width": "1.8",
    "guiShape": "passenger",
    "imgFile": "car_logo.png"  # ensure this file is in the same folder or give full path
})

# Define route
ET.SubElement(routes, "route", {"id": "r_A_C", "edges": route_edges})

# Generate vehicles
for i in range(N):
    vid = f"veh_{i+1}"
    depart_time = f"{i * depart_gap:.1f}"
    ET.SubElement(routes, "vehicle", {
        "id": vid,
        "type": "idm_car",
        "route": "r_A_C",
        "depart": depart_time,
        "departLane": "best",
        "departSpeed": "max"
    })

# Write XML to file
tree = ET.ElementTree(routes)
with open(output_file, "wb") as f:
    f.write(b'<?xml version="1.0" encoding="UTF-8"?>\n')
    tree.write(f, encoding="utf-8", xml_declaration=False)

print(f"Generated {output_file} with {N} vehicles (depart gap {depart_gap}s).")

