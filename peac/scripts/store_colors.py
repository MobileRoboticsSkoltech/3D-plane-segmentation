import sys
import os
import numpy as np
from pypcd import pypcd

if __name__ == '__main__':
    pc = pypcd.PointCloud.from_path(sys.argv[1])
    colors =  pc.pc_data["rgb"]
    colors_map = {}
    for n, color in enumerate(np.unique(colors)):
        colors_map[color] = n
    mapped_colors = np.copy(colors)
    for k, v in colors_map.items(): 
        mapped_colors[colors==k] = v
    filename = sys.argv[1].split('.')[0]
    np.save(filename + ".npy", mapped_colors)