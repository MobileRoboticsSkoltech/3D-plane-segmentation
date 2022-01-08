#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_registered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  int start_index = 1, end_index = 20;
  std::string prefix("/media/work/datasets/BRL/unorganized/scan");
  std::string source_pcd_file, target_pcd_file, source_pose_file, target_pose_file;

  map_cloud->points.clear();

//  for (int index = start_index; index < end_index; index++)
//  {
//    char buf[4];
//    doubletmp;
//    sprintf(buf, "%03d", index);
//    target_pcd_file = prefix + std::string(buf) + ".pcd";
//    target_pose_file = prefix + std::string(buf) + ".pose";
//    doubletarget_odometry[3];
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd_file, *target_cloud) == -1) //* load the file
//    {
//      PCL_ERROR ("Couldn't read file %s!\n", target_pcd_file.c_str());
//      return (-1);
//    }
//    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
//              target_cloud->points.size(), target_cloud->width, target_cloud->height, target_pcd_file.c_str());
//    std::ifstream target_pose_in (target_pose_file.c_str());
//    target_pose_in >> target_odometry[0] >> target_odometry[1] >> tmp >> tmp >> tmp >> target_odometry[2];
//    PCL_INFO ("target odometry (x, y, theta): (%f, %f, %f)!\n", target_odometry[0], target_odometry[1], target_odometry[2]);
//    if (index == start_index)
//      map_cloud->points.insert(map_cloud->points.begin(),
//                               target_cloud->points.begin(), target_cloud->points.end());
//
//    sprintf(buf, "%03d", index + 1);
//    source_pcd_file = prefix + std::string(buf) + ".pcd";
//    source_pose_file = prefix + std::string(buf) + ".pose";
//    doublesource_odometry[3];
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_pcd_file, *source_cloud) == -1) //* load the file
//    {
//      PCL_ERROR ("Couldn't read file %s!\n", source_pcd_file.c_str());
//      return (-1);
//    }
//    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
//              source_cloud->points.size(), source_cloud->width, source_cloud->height, source_pcd_file.c_str());
//    std::ifstream source_pose_in(source_pose_file.c_str());
//    source_pose_in >> source_odometry[0] >> source_odometry[1] >> tmp >> tmp >> tmp >> source_odometry[2];
//    PCL_INFO ("source odometry (x, y, theta): (%f, %f, %f)!\n", source_odometry[0], source_odometry[1], source_odometry[2]);
//
//    Eigen::Matrix4d guess(Eigen::Matrix4d::Zero());
//    doubletheta = (source_odometry[2] - target_odometry[2]) * 3.1415926 / 180;
//    guess(0,0) = cos(theta);
//    guess(0,1) = -sin(theta);
//    guess(1,0) = sin(theta);
//    guess(1,1) = cos(theta);
//    guess(2,2) = 1;
//    guess(3,3) = 1;
//    guess(0,3) = source_odometry[0] - target_odometry[0];
//    guess(1,3) = source_odometry[1] - target_odometry[1];
//
//    pcl::transformPointCloud(*source_cloud, *source_cloud_registered, guess);
//    pcl::visualization::PCLVisualizer viewer ("visualization for translation");
//    viewer.addPointCloud (target_cloud, "target_cloud", 0);
//    viewer.addPointCloud (source_cloud_registered, "source_cloud_registered", 0);
//    viewer.addCoordinateSystem (1.0);
//    viewer.setBackgroundColor (1,1,1);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud_registered");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "target_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "source_cloud_registered");
//    viewer.spin ();
//  }

  for (int index = start_index; index <= end_index; index++)
  {
    char buf[4];
    double tmp;
    sprintf(buf, "%03d", index);
    target_pcd_file = prefix + std::string(buf) + ".pcd";
    target_pose_file = prefix + std::string(buf) + ".pose";
    double target_odometry[3];
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd_file, *target_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", target_pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              target_cloud->points.size(), target_cloud->width, target_cloud->height, target_pcd_file.c_str());
    std::ifstream target_pose_in (target_pose_file.c_str());
    target_pose_in >> target_odometry[0] >> target_odometry[1] >> tmp >> tmp >> tmp >> target_odometry[2];
    PCL_INFO ("target odometry (x, y, theta): (%f, %f, %f)!\n", target_odometry[0], target_odometry[1], target_odometry[2]);
    Eigen::Matrix4f transformation(Eigen::Matrix4f::Zero());
    double theta = target_odometry[2] * 3.14 / 180;
    transformation(0,0) = cos(theta);
    transformation(0,1) = -sin(theta);
    transformation(1,0) = sin(theta);
    transformation(1,1) = cos(theta);
    transformation(2,2) = 1;
    transformation(3,3) = 1;
    transformation(0,3) = target_odometry[0];
    transformation(1,3) = target_odometry[1];
    pcl::transformPointCloud(*target_cloud, *target_cloud, transformation);
    map_cloud->points.insert(map_cloud->points.end(),
                             target_cloud->points.begin(),
                             target_cloud->points.end());
  }

  map_cloud->width = map_cloud->points.size();
  map_cloud->height = 1;
  pcl::io::savePCDFileBinary("map.pcd", *map_cloud);
  return 0;
}






