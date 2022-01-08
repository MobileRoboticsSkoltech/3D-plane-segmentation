#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <Eigen/Core>
#include <iostream>

#include "common/common.h"

int
main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: show <direction> <prefix> <start> <end>.  \n";
    return -1;
  }

  std::string prefix = std::string(argv[1]) + std::string(argv[2]);
  int start_index = atoi(argv[3]);
  int end_index = atoi(argv[4]);

  std::ofstream poses;
  poses.open ("poses");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "show registered clouds");
  p->setBackgroundColor (1,1,1);
  bool downsample = false;
  std::string pcd_file, pose_file, map_file;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

  for (int index = start_index; index <= end_index; index++)
  {
    char buf[4];

    sprintf(buf, "%03d", index);
    pcd_file = prefix + std::string(buf) + ".pcd";
    pose_file = prefix + std::string(buf) + ".pose";

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    std::ifstream pose(pose_file.c_str());
    pose
        >> transformation(0,0) >> transformation(0,1) >> transformation(0,2) >> transformation(0,3)
        >> transformation(1,0) >> transformation(1,1) >> transformation(1,2) >> transformation(1,3)
        >> transformation(2,0) >> transformation(2,1) >> transformation(2,2) >> transformation(2,3)
        >> transformation(3,0) >> transformation(3,1) >> transformation(3,2) >> transformation(3,3);
    std::cout << "transformation: \n" << transformation << std::endl;

    Eigen::Matrix3d rotation;
    rotation.block<3,3>(0,0) = transformation.cast<double> ().block<3,3>(0,0);
    double roll, pitch, yaw;
    tams::gerRPYFromRotationMatrix (rotation, roll, pitch, yaw);
    poses << transformation(0,3) << " " << transformation(1,3) << " " << transformation(2,3) << " " <<
             roll << " " << pitch << " " << yaw << std::endl;

    pcl::transformPointCloud(*cloud, *cloud, transformation);
    *map += *cloud;
    p->addPointCloud (cloud, pcd_file, 0);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, pcd_file);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, pcd_file);
    //p->spinOnce (5000);
    p->spin();
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, pcd_file);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, pcd_file);
  }
  map_file = std::string(argv[1]) + std::string("map.pcd");
  pcl::io::savePCDFileBinary(map_file, *map);
  p->spin();
  return 0;
}

