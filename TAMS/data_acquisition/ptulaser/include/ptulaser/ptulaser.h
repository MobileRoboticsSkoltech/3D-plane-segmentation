#ifndef PTU_LASER_H
#define PTU_LASER_H
#include "ptu/ptu.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/io.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "hokuyo/hokuyo.h"
#include <string>
#include <vector>
#include <math.h>
#include <unistd.h>



#define UTM_30LX_MIN_ANGLE      -2.2689
#define UTM_30LX_MAX_ANGLE      2.2689
#define FULL_STEP_RES           0.051428583

class PtuLaser
{
public:
  /**\brief Typedefs.*/
  typedef pcl::PointXYZ  PointXYZ;
  typedef pcl::PointXYZI PointXYZI; 
  typedef pcl::PointWithRange PointWithRange;
  typedef pcl::PointCloud<PointXYZ>  PointCloudXYZ;
  typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;  
  typedef pcl::PointCloud<PointWithRange> PointCloudWithRange;
  /**\brief Empty constructor.*/
  PtuLaser();
  ~PtuLaser();
  void LaserConf();
  int PtuConf();
  int get3DScanWithIntensity(PointCloudXYZI::Ptr &cloud,
                             IplImage *& depth_img,
                             IplImage *& intensity_img,
                             const double step_angle,
                             const double pan_range = M_PI,
                             const bool clockwise = true);
//  int get3DScanWithRangeAndIntensity(PointCloudXYZIWithRange::Ptr &cloud,
//                                             IplImage *& depth_img,
//                                             IplImage *& intensity_img,
//                                             const double step_angle,
//                                             const double pan_range = M_PI,
//                                             const bool clockwise = true);
  int get3DScanWithRange(PointCloudWithRange::Ptr &cloud,
       const double step_angle,
       const double pan_range = M_PI,
                         const bool clockwise = true);
  int get3DScan(PointCloudXYZ::Ptr &cloud,
                const double step_angle,
                const double pan_range = M_PI,
                const bool clockwise = true);

  int scanDellBoxes(PointCloudXYZ::Ptr &cloud,
                    const double step_angle,
                    const double pan_range,
                    const bool clockwise = true);

public:
  /**The usb to serial port Baud rate used for ptu-d48e.*/
  int Baudrate;
  /**The ptu is attached to /dev/ttyUSB0 as default.*/
  std::string COMportName;
  /**The file descriptor (fd) of ptu com.*/
  ptu::portstream_fd COMstream;
  /**The hokuyo laser is attached /dev/ttyACM0 as defaut.*/
  std::string ACMportName;
  /**The hokuyo laser class consistent to the SCIP2.0 interface*/
  hokuyo::Laser laser;
  /**A struct for returning configuration from the Hokuyo.*/
  hokuyo::LaserConfig laser_config;
  /**A struct for returning laser readings from the Hokuyo.*/
  hokuyo::LaserScan laser_scan;
  int min_step;
  int max_step;
  bool clockwise;
  std::vector<hokuyo::LaserScan> scans;
  std::vector<double > range_image;

private:
  char status;
  struct timeval ptu_executed_time;
  uint64_t ptu_executed_time_stamp;
};

#endif
