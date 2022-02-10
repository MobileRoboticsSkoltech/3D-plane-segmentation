import sys
import numpy as np
import open3d as o3d

path = 'input/'+sys.argv[1]
pcd = o3d.io.read_point_cloud(path)

size = len(pcd.points)
colors = np.zeros((size,3))
pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.io.write_point_cloud('input/result.ply', pcd, write_ascii=True)