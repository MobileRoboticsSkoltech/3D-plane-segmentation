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

int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr first (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr second (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "show registered clouds");
  p->setBackgroundColor (1,1,1);
  int start_index = 0;
  bool downsample = false;
  std::string prefix("/media/work/datasets/bremen_city/pcd/scan");
  std::string pcd_file, transformation_file;
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

  int index = start_index;
  char buf[4];
  sprintf(buf, "%03d", index);
  pcd_file = prefix + std::string(buf) + ".pcd";
  transformation_file = prefix + std::string(buf) + ".pose";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *first) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
            first->points.size(), first->width, first->height, pcd_file.c_str());
  std::ifstream transformation_in(transformation_file.c_str());
  transformation_in
      >> transformation(0,0) >> transformation(0,1) >> transformation(0,2) >> transformation(0,3)
      >> transformation(1,0) >> transformation(1,1) >> transformation(1,2) >> transformation(1,3)
      >> transformation(2,0) >> transformation(2,1) >> transformation(2,2) >> transformation(2,3)
      >> transformation(3,0) >> transformation(3,1) >> transformation(3,2) >> transformation(3,3);
  transformation_in.close();
  pcl::transformPointCloud(*first, *first, transformation);


  index+=1;
  sprintf(buf, "%03d", index);
  pcd_file = prefix + std::string(buf) + ".pcd";
  transformation_file = prefix + std::string(buf) + ".pose";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *second) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
            second->points.size(), second->width, second->height, pcd_file.c_str());
  transformation_in.open(transformation_file.c_str());
  transformation_in
      >> transformation(0,0) >> transformation(0,1) >> transformation(0,2) >> transformation(0,3)
      >> transformation(1,0) >> transformation(1,1) >> transformation(1,2) >> transformation(1,3)
      >> transformation(2,0) >> transformation(2,1) >> transformation(2,2) >> transformation(2,3)
      >> transformation(3,0) >> transformation(3,1) >> transformation(3,2) >> transformation(3,3);
  transformation_in.close();
  pcl::transformPointCloud(*second, *second, transformation);

  p->addPointCloud (first, "first", 0);
  p->addPointCloud (second, "second", 0);
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "first");
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "first");
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "second");
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "second");

  p->spin();
  return 0;
}

