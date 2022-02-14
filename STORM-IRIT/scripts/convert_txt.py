import sys
import os
import numpy as np

if __name__ == '__main__':
    arr = np.loadtxt(sys.argv[1], delimiter="\n")
    filename = sys.argv[1].split('.')[0]
    np.save(filename + ".npy", arr)
    os.remove(sys.argv[1])