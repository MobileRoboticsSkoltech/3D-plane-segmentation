import sys
import os
import open3d as o3d
from shutil import rmtree

def create_dir(dir_path):
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
    else:
        for file_name in os.listdir(dir_path):
            file_path = os.path.join(dir_path, file_name)
            
            if not os.path.isdir(file_path):
                os.remove(file_path)
                
            else:
                rmtree(file_path)

if __name__ == '__main__':
    
    path = 'input/' + sys.argv[1]
    pcd = o3d.io.read_point_cloud(path)

    pcd.estimate_normals()

    filename = sys.argv[1].split('.')[0]

    folder_path = os.path.join('output', filename)
    create_dir(folder_path)

    range = os.path.join(folder_path, filename + "_range")
    pers = os.path.join(folder_path, filename + "_pers")
    scales = os.path.join(folder_path, filename + "_scales")
    create_dir(range)
    create_dir(pers)
    create_dir(scales)

    o3d.io.write_point_cloud(os.path.join(folder_path, filename+".ply"), pcd, write_ascii=True)