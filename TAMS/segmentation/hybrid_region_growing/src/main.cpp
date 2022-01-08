#include "hybrid_region_growing/hybrid_region_growing.h"
#include "hybrid_region_growing/hybrid_region_growing_parameters.h"
#include <fstream>
int
main (int argc, char **argv)
{
  std::ofstream time;
  time.open("indoorHokuyo_HBRG_sl_3.txt");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  struct timeval tpstart,tpend;
  double timeuse;
  tams::HybridRGSegmentationParameters parameters;
  parameters.max_mass2plane_dis = 0.05;
  parameters.min_dot_product = 0.95;
  parameters.max_segment_mse = 0.0008;
  parameters.min_segment_size = 50;
  parameters.subwindow_side_length = 3;
  std::string prefix ("/media/exchange/datasets/BRL/organized/scan");
  /*****************************initialize point cloud in plane extraction*********************************/
  for (int scan_index = 337; scan_index < 400; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    if (pcl::io::loadPCDFile (pcd_file, cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd");
    }
    PCL_INFO ("Loaded %d data points from %s.\n",cloud.size(), pcd_file.c_str());
    time << cloud.size() << " ";
    for (int i = 0; i < 1; i++)
    {
      tams::HybridRGSegmentation segmentation;
      segmentation.setInput (cloud.makeShared());
      segmentation.setparameters (parameters);
      segmentation.preprocessing();
      gettimeofday(&tpstart,NULL);
      segmentation.applySegmentation();
      gettimeofday(&tpend,NULL);
      timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
      timeuse/=1000000;
      time  << timeuse << " ";
      if (true)
      {
        segmentation.colorEncoding(output, true);
        pcl::visualization::PCLVisualizer viewer ("Detected planes with Pseudo-color");
        viewer.addPointCloud (output, "segments");
        viewer.setBackgroundColor (1.0, 1.0, 1.0);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segments");
        viewer.spin ();
      }
    }
    time << std::endl;
  }
  return (0);
}
