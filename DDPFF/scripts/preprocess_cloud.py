import sys
import open3d as o3d


if __name__ == '__main__':
    
    path = 'input/' + sys.argv[1]
    pcd = o3d.io.read_point_cloud(path)

    pcd.paint_uniform_color([0, 0, 0])

    o3d.io.write_point_cloud('result.ply', pcd, write_ascii=True)