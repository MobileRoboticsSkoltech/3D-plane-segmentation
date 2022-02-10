import sys
import numpy as np
import open3d as o3d


path = 'input/result.ply'
pcd = o3d.io.read_point_cloud(path)

planes = []                
with open('output/planes.txt', 'r') as f:
    for line in f:
        planes.append([int(x) for x in line.split()])
    
colors = np.array(pcd.colors)

for ind in planes:
    col = np.random.uniform(0,1, size=(1,3))
    for i in ind:
        colors[i] = col

pcd.colors = o3d.utility.Vector3dVector(colors)

filename = sys.argv[1].split('.')[0]

o3d.io.write_point_cloud('output/'+filename+'_result.pcd', pcd)