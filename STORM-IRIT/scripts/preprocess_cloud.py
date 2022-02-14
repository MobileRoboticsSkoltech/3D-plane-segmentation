import sys
import os
import open3d as o3d
from shutil import rmtree

def create_dir(dir_path):
    if os.path.exists(dir_path):
        rmtree(dir_path)
    os.mkdir(dir_path)

if __name__ == '__main__':
    
    path = 'input/' + sys.argv[1]
    pcd = o3d.io.read_point_cloud(path)

    pcd.estimate_normals()

    filename = sys.argv[1].split('.')[0]

    folder_path = os.path.join('output', filename)
    create_dir(folder_path)

    o3d.io.write_point_cloud(filename + ".ply", pcd, write_ascii=True)