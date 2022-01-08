#include "subwindow_region_growing/subwindow_region_growing.h"
#include "subwindow_region_growing/subwindow_region_growing_parameters.h"
#include <fstream>
int
main (int argc, char **argv)
{
  std::ofstream time;
  time.open("sidelength_3.txt");
  struct timeval tpstart,tpend;
  double timeuse;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  tams::SubwindowRGSegmentationParameters parameters;
  parameters.max_mass2plane_dis = 0.035;
  parameters.min_dot_product = 0.95;
  parameters.max_segment_mse = 0.0008;
  parameters.min_segment_size = 50;
  parameters.subwindow_side_length = 3;
  std::string prefix ("/media/exchange/datasets/indoorHokuyo/scan");
  /*****************************initialize point cloud in plane extraction*********************************/
  for (int scan_index = 26; scan_index < 54; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    if (pcl::io::loadPCDFile (pcd_file, cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd");
    }
    PCL_INFO ("Loaded %d data points from %s.\n",cloud.size(), pcd_file.c_str());
    for (int i = 0; i < 1; i++)
    {
      tams::SubwindowRGSegmentation segmentation;
      segmentation.setparameters (parameters);
      segmentation.setInput (cloud.makeShared());
      segmentation.preprocessing();
      segmentation.applySegmentation();
      if (true)
      {
        segmentation.randomColours(output, true);
        pcl::visualization::PCLVisualizer viewer ("Detected planes with Pseudo-color");
        viewer.addPointCloud (output, "segments");
        viewer.setBackgroundColor (1.0, 1.0, 1.0);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segments");
        viewer.spin ();
      }
    }
  }
  return (0);
}
