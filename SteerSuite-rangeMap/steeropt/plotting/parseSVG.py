"""

    Dependencies:
    1. svg.path 
    pip install svg.path
"""


from xml.dom import minidom
from svg.path import parse_path
import sys
import numpy as np


file = open(sys.argv[1], "r")
svg_string = file.read()
file.close()

scale = 0.5
_distance_threshold = 0.5 * scale
svg_dom = minidom.parseString(svg_string)

path_strings = [path.getAttribute('d') for path in svg_dom.getElementsByTagName('path')]
edges = []
verts = []

def distance(a, b):
    a=(a - b)
    d = np.sqrt((a*a).sum(axis=0))
    return d

def addVert(verts, point_):
    # check if vert already in verts
    out=0
    distance_threshold = 0.25 * scale
    for p in range(len(verts)):
        if (distance(verts[p], point_) < distance_threshold):
            return p
        
    verts.append(point_);
    return len(verts)-1
        
paths__ = len(path_strings)
for j in range(len(path_strings)):
    try:
        path_data = parse_path(path_strings[j])
    except:
        continue
    # print path_data
    for i in range(len(path_data)):
        point = path_data[i]
        # point.start.real = X, point.start.imag =Y
        # print i, point.start.real, point.start.imag, point.start
        v0_ = np.array([point.start.real, point.start.imag]) * scale
        v1_ = np.array([point.end.real, point.end.imag]) * scale
        v0 = addVert(verts, v0_)
        v1 = addVert(verts, v1_)
        if ( distance(v0_, v1_) > _distance_threshold):
            edges.append([v0, v1])
        
    ## for testing file writing
    # if j > 120:
     #    break
    #  now use methods provided by the path_data object
    #  e.g. points can be extracted using 
    #  point = path_data.pos(pos_val) 
    #  where pos_val is anything between 0 and 1
    print ("Number of verts: " + str(len(verts)))
    print ("On path ", j, " of ", paths__)
    
max_x=-10000000
min_x=10000000
max_z=-10000000
min_z=10000000
for vert in verts:
    if (vert[0] > max_x):
        max_x = vert[0]
    if (vert[1] > max_z):
        max_z = vert[1]
        
    if (vert[0] < min_x):
        min_x = vert[0]
    if (vert[1] < min_z):
        min_z = vert[1]    


delta_x = float(max_x + min_x) / 2.0
delta_z = float(max_z + min_z) / 2.0 
print (max_x, min_x, max_z, min_z)
graph_file = open("out.graph", "w")
for vert in verts:
    graph_file.write("v " + str(vert[0]-delta_x) + " 0 " + str(vert[1]-delta_z) + "\n" )
    
graph_file.write("\n")
### Creates file of 100 edges at a time.
### This helps with Revit not crashing??
edge_group_num=250
graph_file_ = None
for e in range(len(edges)):
    ### Open a new graph file and write all nodes/verts to it
    if ((e == 0) or ((e % edge_group_num) == 0)):
        if ((graph_file_ != None) and (not graph_file_.closed)):
            graph_file_.close()
        graph_file_ = open("out"+str(e)+".graph", "w")
        for vert in verts:
            graph_file_.write("v " + str(vert[0]-delta_x) + " 0 " + str(vert[1]-delta_z) + "\n" )
            
        graph_file_.write("\n")
    ### Add some edges to these nodes
    edge = edges[e]
    graph_file_.write("e " + str(edge[0]) + " " + str(edge[1]) + "\n" )
    graph_file.write("e " + str(edge[0]) + " " + str(edge[1]) + "\n" )
    
graph_file.close()
graph_file_.close()
    
