import json
import matplotlib.pyplot as plt
import numpy as np
import graphviz

file_name = "/home/infinity/Code/ros/graph.json"

dot = graphviz.Digraph('G', engine="neato", format="svg")

with open(file_name) as file:
    graph = json.load(file)

for node in graph["nodes"]:
    pose = node["pose"]
    dot.node(str(node["id"]), shape="point", color="red", pos=f"{pose[0]},{pose[1]}!")

# for node in graph["scan_nodes"]

for edge in graph["edges"]:
    dot.edge(str(edge["from"]), str(edge["to"]), arrowhead="none")

dot.render("../data/graph", view=False)  
