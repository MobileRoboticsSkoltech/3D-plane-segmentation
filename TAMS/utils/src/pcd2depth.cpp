#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <time.h>
#include <math.h>
int
main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::string prefix ("/media/work/datasets/crashedCarPark/reordered/scan");
  /*****************************initialize point cloud in plane extraction*********************************/
  for (int scan_index = 7; scan_index <= 32; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    std::string depth_file = "./scrapMetal_range_" + std::string (buf) + ".png";
    char *str = const_cast<char*>(depth_file.c_str());
    if (pcl::io::loadPCDFile (pcd_file, cloud) == -1)
    {
      PCL_ERROR("Couldn't read file test_pcd.pcd");
    }
    PCL_INFO("Loaded %d data points from %s.\n", cloud.size(), pcd_file.c_str());
    int height = cloud.height;
    int width = cloud.width;

    CvSize cs;
    cs.height = cloud.height;
    cs.width = cloud.width;
    IplImage * depth = cvCreateImage (cs, IPL_DEPTH_32F, 1);

    int idx;
    double range;
    double max_range = 0;
    for (size_t i = 0; i < cloud.size(); i++)
    {
      if (!pcl::isFinite(cloud.points[i]))
        continue;
      range = cloud.points[i].x * cloud.points[i].x +
          cloud.points[i].y * cloud.points[i].y +
          cloud.points[i].z * cloud.points[i].z;
      if (range > max_range)
        max_range = range;
    }
    max_range = sqrt(max_range);
    for (int i = 0; i < height; i++)
    {
      for (int j = 0; j < width; j++)
      {
        idx = i * width + j;
        if (std::isnan(cloud.points[idx].x))
        {
          cvSetReal2D (depth, i, j, 1.0);
        }
        else
        {
          range = sqrt(cloud.points[idx].x * cloud.points[idx].x +
                       cloud.points[idx].y * cloud.points[idx].y +
                       cloud.points[idx].z * cloud.points[idx].z);
          cvSetReal2D (depth, i, j, sqrt(range/(max_range * 1.1)));
        }
      }
    }
    std::cout << "the maximum range is : " << max_range << " meters!" << std::endl;
    cvConvertScale(depth, depth, 255, 0);
    cvSaveImage(str,depth);
  }
  return (0);
}
