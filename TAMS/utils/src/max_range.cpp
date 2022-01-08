#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int
main (int argc, char** argv)
{
    if (argc == 1)
    {
        std::cout << "usage: max_range <input_pcd>" << std::endl;
    }
    std::string input_pcd(argv[1]);
    pcl::PointCloud<pcl::PointXYZ> input;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pcd, input) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", input_pcd.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points from %s.\n", input.points.size(), input_pcd.c_str ());

    double max_range = 0;
    double range;
    for (int i = 0; i < input.points.size(); i++)
    {
        if (input.points[i].x != 0 || input.points[i].y != 0 || input.points[i].z != 0)
        {
           range = sqrt(input.points[i].x * input.points[i].x + input.points[i].y * input.points[i].y + input.points[i].z * input.points[i].z);
           if (range > max_range)
               max_range = range;
        }
    }

    std::cerr << "Maximum range in the cloud: " << max_range << std::endl;

    return (0);
}
