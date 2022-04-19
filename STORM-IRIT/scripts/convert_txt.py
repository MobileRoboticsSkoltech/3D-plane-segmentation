import sys
import os
import numpy as np

if __name__ == '__main__':
    if not ("ex_time" in sys.argv[1]):
        arr = np.loadtxt(sys.argv[1], delimiter="\n")
        arr += 1
        filename = sys.argv[1].split('.')[0]
        np.save(filename + ".npy", arr)
        os.remove(sys.argv[1])