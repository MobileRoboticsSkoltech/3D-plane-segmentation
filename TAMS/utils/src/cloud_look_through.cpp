#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <stdlib.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
int
main (int argc, char** argv)
{
    if (argc != 5)
    {
        std::cout << "usage: cloud_look_through <input dir> <prefix> <first index> <last index>" << std::endl;
        return (-1);
    }
    std::string input_dir(argv[1]);
    std::string prefix(argv[2]);
    int first_index = atoi(argv[3]);
    int last_index = atoi(argv[4]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer (argc, argv, "show cloud");
    p->setBackgroundColor (1,1,1);

    for (int index = first_index; index <= last_index; index++)
    {
      char buf[4];
      sprintf(buf,"%03d", index);
      std::string input_pcd = input_dir + prefix + std::string (buf) + ".pcd";

      if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pcd, *cloud) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file %s!\n", input_pcd.c_str());
        return (-1);
      }
      PCL_INFO ("Loaded %d points from %s.\n", cloud->points.size(), input_pcd.c_str ());

      p->addPointCloud (cloud, input_pcd, 0);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, input_pcd);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, input_pcd);
      p->spin();
      p->removePointCloud(input_pcd,0);
    }

    return (0);
}





