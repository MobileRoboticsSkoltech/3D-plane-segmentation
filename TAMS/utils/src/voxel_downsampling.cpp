#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>
int
main (int argc, char** argv)
{
    if (argc == 1)
    {
        std::cout << "usage: voxel_downsampling <input> <output> <voxel_size>" << std::endl;
    }
    std::string input_pcd(argv[1]);
    std::string output_pcd(argv[2]);
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::PointCloud<pcl::PointXYZ> output;
    double voxel_size = atof(argv[3]);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pcd, input) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", input_pcd.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points from %s.\n", input.points.size(), input_pcd.c_str ());

    std::cerr << "PointCloud size before filtering: " << input.size() << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input.makeShared());
    sor.setLeafSize (voxel_size, voxel_size, voxel_size);
    sor.filter (output);

    std::cerr << "PointCloud size after filtering: " << output.width * output.height << std::endl;
    pcl::io::savePCDFileBinary(output_pcd, output);

    return (0);
}
