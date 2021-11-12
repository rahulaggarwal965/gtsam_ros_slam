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
    dot.node(str(node["id"]), shape="point", pos=f"{pose[0] * 10},{pose[1] * 10}!")

for edge in graph["edges"]:
    dot.edge(str(edge["from"]), str(edge["to"]), arrowhead="none")

dot.render("output/graph", view=False)  
