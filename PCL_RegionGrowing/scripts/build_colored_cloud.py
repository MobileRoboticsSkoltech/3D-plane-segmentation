import numpy as np
import os
import open3d as o3d


def create_colored_cloud():
    rootdir = "../input"

    for subdir, _, files in os.walk(rootdir):
        for file in files:
            file_path = os.path.join(subdir, file)
            npy_path = os.path.join("../output", file[:-4], file[:-4] + ".npy")
            labels = np.load(npy_path)
            colors = np.zeros((labels.shape[0], 3), np.float64)
            unique_labels = np.unique(labels)
            pcd = o3d.io.read_point_cloud(file_path)
            pcd.paint_uniform_color([0, 0, 0])
            set_of_colors = set()
            dict_of_colors = {}

            for unique_label in unique_labels:
                col = np.random.uniform(0, 1, size=(1,3))

                while tuple(col[0]) in set_of_colors:
                    col = np.random.uniform(0, 1, size=(1,3))

                set_of_colors.add(tuple(col[0]))
                dict_of_colors[unique_label] = tuple(col[0])

            for index in range(labels.shape[0]):
                colors[index] = dict_of_colors[labels[index]]

            pcd.colors = o3d.utility.Vector3dVector(colors)
            o3d.io.write_point_cloud(os.path.join("../output", file[:-4], file[:-4] + ".pcd"), pcd)


if __name__ == "__main__":
    create_colored_cloud()
