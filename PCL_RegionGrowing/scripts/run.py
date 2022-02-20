import os

from build_colored_cloud import create_colored_cloud
from save_as_npy import convert_txt_files_to_npy_format

if __name__ == "__main__":
    os.system("build/RegionGrowing ./input")
    convert_txt_files_to_npy_format()
    create_colored_cloud()
