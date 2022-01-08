#include <iostream>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <stdlib.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>

int
main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointT;
  typedef typename pcl::PointCloud<PointT> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;
  typedef PointCloud::ConstPtr PointCloudConstPtr;

  typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
  typedef ColorHandler::Ptr ColorHandlerPtr;
  typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

  if (argc != 2)
  {
      std::cout << "usage: map_browser <map cloud_xyz>" << std::endl;
      return (-1);
  }
  std::string map_file(argv[1]);
  PointCloudPtr cloud_xyz (new PointCloud);
  ColorHandlerPtr color_handler;
  pcl::visualization::PCLVisualizer *p;
  pcl::PCDReader reader;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "show cloud_xyz");
  p->setBackgroundColor (1,1,1);

  Eigen::Vector4f origin(-50, -50, 50, 1);
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

  sensor_msgs::PointCloud2::Ptr cloud;
  cloud.reset (new sensor_msgs::PointCloud2);
  int version;
  if (reader.read (map_file, *cloud, origin, orientation, version) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", map_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points from %s.\n", cloud->width * cloud->height, map_file.c_str());


  pcl::fromROSMsg (*cloud, *cloud_xyz);
  doublemin_x = 1000, max_x = -1000, min_y = 1000, max_y = -1000, min_z = 1000, max_z = -1000;
  for (size_t j =0; j < cloud_xyz->size(); j++)
  {
    if (cloud_xyz->points[j].x < min_x)
      min_x = cloud_xyz->points[j].x;
    if (cloud_xyz->points[j].y < min_y)
      min_y = cloud_xyz->points[j].y;
    if (cloud_xyz->points[j].z < min_z)
      min_z = cloud_xyz->points[j].z;
    if (cloud_xyz->points[j].x > max_x)
      max_x = cloud_xyz->points[j].x;
    if (cloud_xyz->points[j].y > max_y)
      max_y = cloud_xyz->points[j].y;
    if (cloud_xyz->points[j].z > max_z)
      max_z = cloud_xyz->points[j].z;
  }

  doubletranslation_x = (min_x + max_x) * 0.5 - 5;
  doubletranslation_y = (min_y + max_y) * 0.5;
  doubletranslation_z = (min_z + max_z) * 0.5;

  for (size_t k = 0; k < cloud_xyz->size(); k++)
  {
    cloud_xyz->points[k].x -= translation_x;
    cloud_xyz->points[k].y -= translation_y;
  }

  PCL_INFO ("min x: %f, max x: %f, min y: %f, max y: %f, min_z: %f, max z: %f.\n", min_x, max_x, min_y, max_y, min_z, max_z);

  pcl::io::savePCDFileBinary("output.pcd", *cloud_xyz);
  Eigen::Matrix3d rotation;
  rotation = orientation;
  p->setCameraPose (origin [0]                  , origin [1]                  , origin [2],
                    origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2),
                    rotation (0, 1),              rotation (1, 1),              rotation (2, 1));
  for (size_t f = 0; f < cloud->fields.size (); ++f)
  {
    if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba")
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud));
    else
    {
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> (cloud, cloud->fields[f].name));
    }
    p->addPointCloud (cloud, color_handler, origin, orientation, map_file, 0);
  }
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, map_file);
  p->addCoordinateSystem(1.0, 0);
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, map_file);
  p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, map_file);

  p->camera_.fovy = 0.5;
  p->camera_.clip[1] = 10.0;
  p->updateCamera();
  while(!p->wasStopped())
  {
    p->setCameraPose(-50, 0, 25, 0.0, 0.0, 10.0, 0.0, 0.0, 1.0);
    std::cout << "vertical field of view: " << p->camera_.fovy << std::endl;
    std::cout << "clip: " << p->camera_.clip[0] << ", " << p->camera_.clip[1] << std::endl;
    std::cout << "camera pos: " << p->camera_.pos[0] << ", " << p->camera_.pos[1] << ", " << p->camera_.pos[2] << std::endl;
    std::cout << "look at: " << p->camera_.focal[0] << ", " << p->camera_.focal[1] << ", " << p->camera_.focal[2] << std::endl;
    std::cout << "up vector: " << p->camera_.view[0] << ", " << p->camera_.view[1] << ", " << p->camera_.view[2] << std::endl;
    p->spinOnce(3000);
  }

  p->spin();

  return (0);
}
