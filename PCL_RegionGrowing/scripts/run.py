import os

from PCL_RegionGrowing.scripts.build_colored_cloud import create_colored_cloud
from PCL_RegionGrowing.scripts.save_as_npy import convert_txt_files_to_npy_format

if __name__ == "__main__":
    os.system("../cmake-build-debug/RegionGrowing ../input/")
    convert_txt_files_to_npy_format()
    create_colored_cloud()
