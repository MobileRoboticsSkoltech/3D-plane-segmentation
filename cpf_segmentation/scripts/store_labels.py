import sys
import os
import numpy as np
from pypcd import pypcd

if __name__ == '__main__':
    pc = pypcd.PointCloud.from_path(sys.argv[1])
    arr =  pc.pc_data["label"]
    filename = sys.argv[1].split('.')[0]
    np.save(filename + ".npy", arr)