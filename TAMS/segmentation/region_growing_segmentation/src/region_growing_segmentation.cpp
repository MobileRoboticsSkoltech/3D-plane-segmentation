#include "region_growing_segmentation/region_growing_segmentation.h"
#include "region_growing_segmentation/impl/region_growing_segmentation.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
int
main(int argc, char **argv)
{
  using namespace tams;
  tams::RGSegmentation<pcl::PointXYZ> segmenter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", argv[1]);
    return (-1);
  }
  PlanarSegment::StdVector segments;
  segmenter.setInputCloud(cloud);
  segmenter.segmentation(output);
  segmenter.saveTimes();
  segmenter.getSegments(segments);
  for (size_t i = 0; i < segments.size(); i++)
  {
    for (size_t j = i; j < segments.size(); j++)
    {
      if (segments[i].area < segments[j].area)
      {
        PlanarSegment tmp = segments[i];
        segments[i] = segments[j];
        segments[j] = tmp;
      }
    }
  }
  for (size_t i = 0; i < segments.size(); i++)
  {
    std::cout << segments[i].area << std::endl;
  }
  pcl::visualization::PCLVisualizer viewer("Detected planes with Pseudo-color");
  viewer.setBackgroundColor(0.0,0.0,0.0);
  viewer.addPointCloud(output,"point cloud", 0);
  viewer.addCoordinateSystem();
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"point cloud");
  for (size_t i = 0; i < segments.size(); i++)
  {
    std::ostringstream area;
    area << segments[i].area;
    pcl::PointXYZ position;
    position.x = segments[i].mass_center(0);
    position.y = segments[i].mass_center(1);
    position.z = segments[i].mass_center(2);
    viewer.addText3D ( area.str(), position, 0.1);
  }
  viewer.spin ();
  return (0);
}
