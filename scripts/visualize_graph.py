import json
import matplotlib.pyplot as plt
import numpy as np
import graphviz
import math

def normalize_pose(pose):
    new_theta = pose[2]
    if new_theta > 2 * math.pi:
        new_theta -= 2 * math.pi
    if new_theta <= 0:
        new_theta += 2 * math.pi
    return [pose[0], pose[1], new_theta]

def add_relative_pose(absolute, relative):
    new_theta = absolute[2] + relative[2]
    if new_theta > math.pi:
        new_theta -= 2 * math.pi
    if new_theta < -math.pi:
        new_theta += 2 * math.pi

    delta_x = math.cos(absolute[2]) * relative[0] - math.sin(absolute[2]) * relative[1]
    delta_y = math.sin(absolute[2]) * relative[0] + math.cos(absolute[2]) * relative[1]

    return normalize_pose([absolute[0] + delta_x, absolute[1] + delta_y, new_theta])


file_name = "/home/infinity/Code/ros/graph.json"

dot = graphviz.Digraph('G', engine="neato", format="svg")

with open(file_name) as file:
    graph = json.load(file)

for node in graph["nodes"]:
    pose = node["pose"]
    dot.node(f"Odometry{node['id']}", str(node["id"]), 
            pos=f"{pose[0] * 10},{pose[1] * 10}!",
            shape="point",
            color="blue",
            fixedsize="true",
            width="0.4",
            height="0.3")
 
for node in graph["scan_nodes"]:
    pose = node["pose"]
    dot.node(f"Scan{node['id']}", str(node["id"]),
            pos=f"{pose[0] * 10},{pose[1] * 10}!",
            shape="point",
            color="green",
            fixedsize="true",
            width="0.4",
            height="0.3")

for edge in graph["edges"]:
    start = edge["from"]
    end = edge["to"]
#     color = "red" if edge["type"] == 0 else "purple"
#     abs_pose = graph["nodes"][start]["pose"]
#     check = add_relative_pose(abs_pose, edge["relative_pose"])
#     dot.node(f"Check{end}", str(end), 
#             pos=f"{check[0] * 10},{check[1] * 10}!",
#             shape="point",
#             color=color,
#             fixedsize="true",
#             width="0.4",
#             height="0.3")
# 
    dot.edge(f"Odometry{start}", f"Odometry{end}", arrowhead="none")

dot.render("../data/graph", view=False)  
