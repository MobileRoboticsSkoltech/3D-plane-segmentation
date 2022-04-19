import os
import numpy as np


def convert_txt_files_to_npy_format():
    rootdir = "./output"

    for subdir, _, files in os.walk(rootdir):
        for file in files:
            if "ex_time" in file:
                continue
            file_path = os.path.join(subdir, file)
            labels_array = np.loadtxt(file_path).astype(int)

            np.save(file_path[:-4] + ".npy", labels_array)
            os.remove(file_path)
