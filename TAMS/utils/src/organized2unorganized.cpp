#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>

int
main (int argc, char** argv)
{
    if (argc != 5)
    {
        std::cout << "usage: organized2unorganized <input dir> <prefix> <first index> <last index>" << std::endl;
	return (-1);
    }
    std::string input_dir(argv[1]);
    std::string prefix(argv[2]);
    int first_index = atoi(argv[3]);
    int last_index = atoi(argv[4]);  

    pcl::PointCloud<pcl::PointXYZ>::Ptr input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
	
    for (int index = first_index; index <= last_index; index++)
    {
      char buf[4];
      sprintf(buf,"%03d", index);
      std::string input_pcd = input_dir + prefix + std::string (buf) + ".pcd";
      std::string output_pcd = "./" + prefix + std::string(buf) + ".pcd";

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pcd, *input) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file %s!\n", input_pcd.c_str());
        return (-1);
      }
      PCL_INFO ("Loaded %d points from %s.\n", input->points.size(), input_pcd.c_str ());

      output->resize(input->size());
      int cnt = 0;
      for (int i = 0; i < input->size(); i++)
      {
        if ((std::isnan(input->points[i].x) || std::isnan(input->points[i].y) || std::isnan(input->points[i].z)))
          continue;
        output->points[cnt] = input->points[i];
        cnt++;
      }
      PCL_INFO("There are %d valid points in %s.\n", cnt, input_pcd.c_str());
      output->points.erase(output->points.begin() + cnt, output->points.end());
      output->width = cnt;
      output->height = 1;
      pcl::io::savePCDFileBinary (output_pcd, *output);
    }

    return (0);
}
