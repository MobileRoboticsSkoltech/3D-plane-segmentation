import sys
import os
import open3d as o3d

if __name__ == '__main__':
    pcd = o3d.io.read_point_cloud(sys.argv[1])
    filename = sys.argv[1].split('.')[0]
    o3d.io.write_point_cloud(filename + ".pcd", pcd, write_ascii=True)
    os.remove(sys.argv[1])