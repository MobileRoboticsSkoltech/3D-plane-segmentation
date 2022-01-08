#include "segments_descriptor/segments_descriptor.h"
#include "segments_descriptor/impl/segments_descriptor.hpp"
#include "region_growing_segmentation/region_growing_segmentation.h"
#include "region_growing_segmentation/impl/region_growing_segmentation.hpp"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/point_cloud_handlers.h>

int
main(int argc, char **argv)
{
  using namespace tams;
  std::ofstream of;
  //RGSegmentation<tamrot::PointXYZIWithRange> segmenter;
  //SegmentsDescriptor<PointXYZIWithRange> segments_descriptor;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<tamrot::PointXYZIWithRange>::Ptr cloud(new pcl::PointCloud<tamrot::PointXYZIWithRange>);
  RGSegmentation<pcl::PointXYZ> segmenter;
  SegmentsDescriptor<pcl::PointXYZ> segments_descriptor;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string prefix = "../../../datasets/test/xyzdi00";
  std::vector<std::vector<FeatureBasedOnSegment> > appearances_array;
  std::vector<FeatureBasedOnSegment> appearances;
//  if (pcl::io::loadPCDFile<tamrot::PointXYZIWithRange> ("../../../datasets/campus/xyzdi0000.pcd", *cloud) == -1) //* load the file
//  {
//    PCL_ERROR ("Couldn't read file %s!\n", fileName.c_str());
//    return (-1);
//  }
//  segmenter.setInputCloud(cloud);
//  segmenter.segmentation(output);
//  segments_descriptor.setInputCloud(cloud);
//  segments_descriptor.setSegments(segmenter.getSegments());
//  segments_descriptor.computeAppearances(appearances);

  for (int scan_index = 1; scan_index < 13; scan_index++)
  {
    char *buf = new char[2];
    sprintf(buf,"%02d",scan_index);
    std::string fileName = prefix + std::string (buf) + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", fileName.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, fileName.c_str ());
    segmenter.setInputCloud(cloud);
    segmenter.segmentation(output);
    segments_descriptor.setInputCloud(cloud);
    segments_descriptor.setSegments(segmenter.getSegments());
    segments_descriptor.computeAppearances(appearances);
    appearances_array.push_back(appearances);
  }
  of.open("orientation_histogram_distance");
  for (size_t i = 0; i < appearances_array.size(); i++)
  {
    for (size_t j = 0; j < appearances_array.size(); j++)
    {
      double min = 1000;
      for (size_t u = 0; u < appearances_array[i].size(); u++)
      {
        for (size_t v = 0; v < appearances_array[j].size(); v++)
        {
          double dis = appearances_array[i][u].distance(appearances_array[j][v]);
          if (dis < min)
          {
            min = dis;
          }
        }
      }
      of << min << "\t";
    }
    of << std::endl;
  }
  of.close();

//  pcl::visualization::PCLVisualizer viewer("Detected planes with Pseudo-color");
//  viewer.setBackgroundColor(0.0,0.0,0.0);
//  viewer.addPointCloud(output,"point cloud", 0);
//  //viewer.addCoordinateSystem();
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"point cloud");
//  segments_descriptor.appearancesVisualization(&viewer, appearances);
//  viewer.spin ();
  return (0);
}
