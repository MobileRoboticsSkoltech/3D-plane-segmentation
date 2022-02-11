import sys
import numpy as np
import open3d as o3d


if __name__ == '__main__':

    path = 'result.ply'
    pcd = o3d.io.read_point_cloud(path)

    planes = []                
    with open('planes.txt', 'r') as f:
        for line in f:
            planes.append(np.asarray([int(x) for x in line.split()]))
    
        
    colors = np.array(pcd.colors)
    s = set()
    for plane_indices in planes:
        
        col = np.random.uniform(0, 1, size=(1,3))
        
        while tuple(col[0]) in s:
            col = np.random.uniform(0, 1, size=(1,3))
        
        s.add(tuple(col[0]))

        if plane_indices.size > 0:
            colors[plane_indices] = col

    pcd.colors = o3d.utility.Vector3dVector(colors)

    filename = sys.argv[1].split('.')[0]

    o3d.io.write_point_cloud('output/' + filename + '_result.pcd', pcd)