import numpy as np
import os
import open3d as o3d


def create_colored_cloud():
    rootdir = "./input"

    for file in os.listdir(rootdir):
        if "ex_time" in file:
            continue
        file_path = os.path.join(rootdir, file)
        npy_path = os.path.join("./output", file[:-4], file[:-4] + ".npy")
        labels = np.load(npy_path)
        colors = np.zeros((labels.shape[0], 3), np.float64)
        unique_labels = np.unique(labels)
        pcd = o3d.io.read_point_cloud(file_path)
        pcd.paint_uniform_color([0, 0, 0])
        set_of_colors = set()

        for unique_label in unique_labels:
            col = np.random.uniform(0, 1, size=3)

            while tuple(col) in set_of_colors:
                col = np.random.uniform(0, 1, size=3)

            set_of_colors.add(tuple(col))
            colors[labels == unique_label] = col

        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.io.write_point_cloud(
            os.path.join("./output", file[:-4], file[:-4] + ".pcd"), pcd
        )