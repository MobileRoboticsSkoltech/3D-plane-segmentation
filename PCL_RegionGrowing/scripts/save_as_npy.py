import os
import numpy as np


def convert_txt_files_to_npy_format():
    rootdir = "../output"

    for subdir, _, files in os.walk(rootdir):
        for file in files:
            file_path = os.path.join(subdir, file)
            labels_array = []
            with open(file_path, "r") as file:
                lines = file.readlines()
                for line in lines:
                    labels_array.append(int(line))

            np.save(file_path[:-4] + ".npy", np.array(labels_array))
            os.remove(file_path)