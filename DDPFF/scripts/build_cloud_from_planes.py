import sys
import os
import numpy as np
import open3d as o3d
from shutil import rmtree
if __name__ == '__main__':

    path = 'result.ply'
    pcd = o3d.io.read_point_cloud(path)

    planes = []
    with open('planes.txt', 'r') as f:
        for line in f:
            planes.append(np.asarray([int(x) for x in line.split()]))


    colors = np.array(pcd.colors)
    labels = np.zeros(colors.shape[0], dtype=int)
    s = set()
    for index, plane_indices in enumerate(planes):

        col = np.random.uniform(0, 1, size=(1,3))

        while tuple(col[0]) in s:
            col = np.random.uniform(0, 1, size=(1,3))

        s.add(tuple(col[0]))

        if plane_indices.size > 0:
            colors[plane_indices] = col
            labels[plane_indices] = index + 1

    pcd.colors = o3d.utility.Vector3dVector(colors)

    filename = sys.argv[1].split('.')[0]

    folder_path = os.path.join('output', filename)
    if os.path.exists(folder_path):
        rmtree(folder_path)
    os.mkdir(folder_path)

    np.save(os.path.join(folder_path, "{}.npy".format(filename)), labels)
    os.replace("output/ex_time.txt", os.path.join(folder_path,"ex_time.txt"))
    o3d.io.write_point_cloud(os.path.join(folder_path, "{}.pcd".format(filename)), pcd)
