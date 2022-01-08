/**\brief 3D point cloud acquisition based on PTU-D48E and Hokuyo UTM-30LX.
 * To rotate the 2D laser scanner, only the pan-movement is used. The step
 * mode is set to "full step", which gives resolutions at 0.051428583 degree/step.
 *
 */
#include "ptulaser/ptulaser.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <algorithm>
#include <sys/time.h>

PtuLaser::PtuLaser()
{
  Baudrate = 9600;
  COMportName = "/dev/ttyUSB0";
  ACMportName = "/dev/ttyACM1";
  min_step = 0;
  max_step = 1080;
}
PtuLaser::~PtuLaser()
{
  if (COMstream > 0)
  {
    ptu::close_host_port(COMstream);
  }
  if (laser.portOpen())
  {
    laser.close();
  }
}
void PtuLaser::LaserConf()
{
  /**Open the laser port.*/
  laser.open(ACMportName.c_str());
  /**Turn on the laser.*/
  laser.laserOn();
//  /**Set to high sensitivith mode.*/
//  const char *cmd = "HS1";
//  int res = laser.sendCmd(cmd);
//  if (res == 0)
//      std::cout << "High sensitivity mode has been set!\n";
//  if (res == 1)
//      std::cout << "Parameter error!\n";
//  if (res == 2)
//      std::cout << "Already run on high sensitivity mode!\n";
//  if (res == 3)
//      std::cout << "High sensitivity mode command is incompatible with current sensor model!\n";
}

int PtuLaser::PtuConf()
{
  /**Set the baud rate of ptu-d48e.*/
  ptu::set_baud_rate(Baudrate);
  /**open the ptu serial port.*/
  COMstream = ptu::open_host_port(COMportName.c_str());
  if (COMstream == PORT_NOT_OPENED)
  {
    PCL_ERROR("PTU-D48E serial Port Open Failed!");
    return -1;
  }
  //Just in case there is an ISM on the line, talk directly to PTU.
  ptu::SerialStringOut(COMstream, (unsigned char *)"   ");
  ptu::select_unit(0);
  //ptu::reset_ptu_pan();
  switch ((ptu::reset_PTU_parser(5000)))
  {
    case PTU_OK:
      PCL_DEBUG("Pending commands and input buffer flushed!");
      break;
    case PTU_FIRMWARE_VERSION_TOO_LOW:
      PCL_ERROR("Error(reset_PTU_parser): PTU FIRMWARE VERSION MATCH ERROR!");
      return -1;
    case PTU_NOT_RESPONDING:
      PCL_ERROR("Error(reset_PTU_parser): PTU_NOT_RESPONDING");
      return -1;
  }
  //ptu::reset_ptu_pan();
  signed short int offset = -250;
  ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, ABSOLUTE);
  return 0;
}

int PtuLaser::get3DScanWithIntensity(PointCloudXYZI::Ptr &cloud,
                                     IplImage *& depth_img,
                                     IplImage *& intensity_img,
                                     const double step_angle,
                                     const double pan_range,
                                     const bool clockwise)
{
  scans.clear();
  signed short int offset = static_cast<signed short int> (step_angle / FULL_STEP_RES);
  if (clockwise == false)
  {
    offset = -offset;
  }
  int afrt = laser.afrt();
  long long int time_stamp_offset = laser.getHokuyoClockOffset(10);
  laser.laserOn();
  laser.requestScans(true, min_step, max_step, 1, 0, 0, -1);
  int position = ptu::get_current(PAN, POSITION);
  std::cout << "step number:" << abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset)))) << std::endl;
  for (int step = 0; step < abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset))))+1; step++)
  {
    if ((status = ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, RELATIVE)) == TRUE)
    {
      return -1;
    }
    if ((status = ptu::await_completion()) != PTU_OK)
    {
      return -1;
    }
    gettimeofday(&ptu_executed_time, NULL);
    ptu_executed_time_stamp = (uint64_t)(ptu_executed_time.tv_sec) * 1000000000 + (uint64_t)(ptu_executed_time.tv_usec)
        * 1000 + time_stamp_offset;
    usleep(45000);
    bool isScanLatterThanPtu = false;
    while (!isScanLatterThanPtu)
    {
      laser.serviceScan(laser_scan);
      if (ptu_executed_time_stamp < laser_scan.self_time_stamp)
      {
        isScanLatterThanPtu = true;
        scans.push_back(laser_scan);
      }
    }
  }
  laser.stopScanning();
  cloud->height = scans.size() * 2;
  cloud->width = scans[0].ranges.size() / 2;
  cloud->points.resize(cloud->height * cloud->width);

  CvSize cs;
  cs.height = cloud->height;
  cs.width = cloud->width;
  depth_img = cvCreateImage(cs,IPL_DEPTH_32F, 1);
  intensity_img = cvCreateImage(cs,IPL_DEPTH_32F, 1);

  for (int i = 0; i < static_cast<int> (scans.size()); i++)
  {
    double cos_1 = cos(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double sin_1 = sin(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double cos_2 = -cos_1;
    double sin_2 = -sin_1;
    for (int j = min_step; j < afrt; j++)
    {
      cloud->points[i * cloud->width + afrt - j - 1].x = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * cos_1;
      cloud->points[i * cloud->width + afrt - j - 1].y = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * sin_1;
      cloud->points[i * cloud->width + afrt - j - 1].z = scans[i].ranges[j] * cos((afrt - j) * 0.25 * M_PI / 180);
      PCL_DEBUG("scans[%d].intensities[%d]: %f\n",i,j, scans[i].intensities[j]);
      cloud->points[i * cloud->width + afrt - j - 1].intensity = scans[i].intensities[j];
      cvSetReal2D(depth_img,i,afrt - j - 1,static_cast<double >(scans[i].ranges[j]));
      cvSetReal2D(intensity_img,i,afrt - j - 1,static_cast<double >(scans[i].intensities[j]));
    }
    for (int j = afrt + 1; j <= max_step; j++)
    {
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * cos_2;
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * sin_2;
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = scans[i].ranges[j] * cos((j - afrt) * 0.25 * M_PI / 180);
      PCL_DEBUG("scans[%d].intensities[%d]: %f\n",i,j, scans[i].intensities[j]);
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].intensity = scans[i].intensities[j];
      cvSetReal2D(depth_img,i + scans.size(),j - 1 - afrt,static_cast<double >(scans[i].ranges[j]));
      cvSetReal2D(intensity_img,i + scans.size(),j - 1 - afrt,static_cast<double >(scans[i].intensities[j]));
    }
  }
  double  max_intensity, min_intensity;
  double  max_depth, min_depth;
  cvMinMaxLoc(depth_img, &min_depth, &max_depth);
  cvMinMaxLoc(intensity_img, &min_intensity, &max_intensity);
  cvConvertScale(depth_img, depth_img, 255 / (max_depth - min_depth), -min_depth);
  cvConvertScale(intensity_img, intensity_img, 255 / (max_intensity - min_intensity), -min_intensity);
  
  for(int i=0;i< static_cast<int>(cloud->height);i++)
  {
    for(int j=0;j< static_cast<int>(cloud->width);j++)
    {
      cloud->points[i*cloud->width+j].intensity = (cloud->points[i*cloud->width+j].intensity - min_intensity) / (max_intensity-min_intensity);
      //std::cout<<cloud->points[i*cloud->width+j].intensity<<" ";
    }
  } 

  PCL_INFO("3D scan has been acquired!\n");
  return 0;
}
/*
int PtuLaser::get3DScanWithRangeAndIntensity(PointCloudXYZIWithRange::Ptr &cloud,
                                             IplImage *& depth_img,
                                             IplImage *& intensity_img,
                                             const double step_angle,
                                             const double pan_range,
                                             const bool clockwise)
{
  scans.clear();
  signed short int offset = static_cast<signed short int> (step_angle / FULL_STEP_RES);
  if (clockwise == false)
  {
    offset = -offset;
  }
  int afrt = laser.afrt();
  long long int time_stamp_offset = laser.getHokuyoClockOffset(10);
  laser.laserOn();
  laser.requestScans(true, min_step, max_step, 1, 0, 0, -1);
  int position = ptu::get_current(PAN, POSITION);
  std::cout << "step number:" << abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset)))) << std::endl;
  for (int step = 0; step < abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset))))+1; step++)
  {
    if ((status = ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, RELATIVE)) == TRUE)
    {
      return -1;
    }
    if ((status = ptu::await_completion()) != PTU_OK)
    {
      return -1;
    }
    gettimeofday(&ptu_executed_time, NULL);
    ptu_executed_time_stamp = (uint64_t)(ptu_executed_time.tv_sec) * 1000000000 + (uint64_t)(ptu_executed_time.tv_usec)
        * 1000 + time_stamp_offset;
    usleep(45000);
    bool isScanLatterThanPtu = false;
    while (!isScanLatterThanPtu)
    {
      laser.serviceScan(laser_scan);
      if (ptu_executed_time_stamp < laser_scan.self_time_stamp)
      {
        isScanLatterThanPtu = true;
        scans.push_back(laser_scan);
      }
    }
  }
  laser.stopScanning();
  cloud->height = scans.size() * 2;
  cloud->width = scans[0].ranges.size() / 2;
  cloud->points.resize(cloud->height * cloud->width);

  CvSize cs;
  cs.height = cloud->height;
  cs.width = cloud->width;
  depth_img = cvCreateImage(cs,IPL_DEPTH_32F, 1);
  intensity_img = cvCreateImage(cs,IPL_DEPTH_32F, 1);

  for (int i = 0; i < static_cast<int>(scans.size()); i++)
  {
    double cos_1 = cos(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double sin_1 = sin(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double cos_2 = -cos_1;
    double sin_2 = -sin_1;
    for (int j = min_step; j < afrt; j++)
    { 
      if (scans[i].ranges[j] > 0.1 && scans[i].ranges[j] < 30)
      {
        cloud->points[i * cloud->width + afrt - j - 1].x = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * cos_1;
        cloud->points[i * cloud->width + afrt - j - 1].y = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * sin_1;
        cloud->points[i * cloud->width + afrt - j - 1].z = scans[i].ranges[j] * cos((afrt - j) * 0.25 * M_PI / 180);
        ROS_DEBUG_STREAM("scans["<<i<<"].intensities["<<j<<"]:"<<scans[i].intensities[j]<<std::endl);
        cloud->points[i * cloud->width + afrt - j - 1].range = scans[i].ranges[j];
        cloud->points[i * cloud->width + afrt - j - 1].intensity = scans[i].intensities[j];
        cvSetReal2D(depth_img,i,afrt - j - 1,static_cast<double >(scans[i].ranges[j]));
        cvSetReal2D(intensity_img,i,afrt - j - 1,static_cast<double >(scans[i].intensities[j]));
      }
      else 
      {
	cloud->points[i * cloud->width + afrt - j - 1].x = 0.0;
	cloud->points[i * cloud->width + afrt - j - 1].y = 0.0;
        cloud->points[i * cloud->width + afrt - j - 1].z = 0.0;
        cloud->points[i * cloud->width + afrt - j - 1].range = 0.0;
        cloud->points[i * cloud->width + afrt - j - 1].intensity = 0.0;
        cvSetReal2D(depth_img,i,afrt - j - 1, 0.0);
        cvSetReal2D(intensity_img,i,afrt - j - 1, 0.0);
      }
    }
    for (int j = afrt + 1; j <= max_step; j++)
    {
      if (scans[i].ranges[j] > 0.1 && scans[i].ranges[j] < 30)
      {
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * cos_2;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * sin_2;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = scans[i].ranges[j] * cos((j - afrt) * 0.25 * M_PI / 180);
        ROS_DEBUG_STREAM("scans["<<i<<"].intensities["<<j<<"]:"<<scans[i].intensities[j]<<std::endl);
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].range = scans[i].ranges[j];
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].intensity = scans[i].intensities[j];
        cvSetReal2D(depth_img,i + scans.size(),j - 1 - afrt,static_cast<double >(scans[i].ranges[j]));
        cvSetReal2D(intensity_img,i + scans.size(),j - 1 - afrt,static_cast<double >(scans[i].intensities[j]));
      }
      else
      {
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = 0.0;
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = 0.0;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = 0.0;
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].range = 0.0;
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].intensity = 0.0;
	cvSetReal2D(depth_img,i + scans.size(),j - 1 - afrt, 0.0);
        cvSetReal2D(intensity_img,i + scans.size(),j - 1 - afrt, 0.0);
      }
    }
  }
  double max_intensity, min_intensity;
  double max_depth, min_depth;
  cvMinMaxLoc(depth_img, &min_depth, &max_depth);
  cvMinMaxLoc(intensity_img, &min_intensity, &max_intensity);
  cvConvertScale(depth_img, depth_img, 255 / (max_depth - min_depth), -min_depth);
  cvConvertScale(intensity_img, intensity_img, 255 / (max_intensity - min_intensity), -min_intensity);
  
  ROS_INFO_STREAM("3D scan has been acquired"<<std::endl);
  return 0;
}*/

int PtuLaser::get3DScanWithRange(PointCloudWithRange::Ptr &cloud,
                                 const double step_angle,
                                 const double pan_range,
                                 const bool clockwise)
{
  scans.clear();
  signed short int offset = static_cast<signed short int> (step_angle / FULL_STEP_RES);
  if (clockwise == false)
  {
    offset = -offset;
  }
  int afrt = laser.afrt();
  long long int time_stamp_offset = laser.getHokuyoClockOffset(10);
  laser.laserOn();
  laser.requestScans(false, min_step, max_step, 1, 0, 0, -1);

  int position = ptu::get_current(PAN, POSITION);
  std::cout << "step number:" << abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset)))) << std::endl;
  for (int step = 0; step < abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset))))+1; step++)
  {
    if ((status = ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, RELATIVE)) == TRUE)
    {
      return -1;
    }
    if ((status = ptu::await_completion()) != PTU_OK)
    {
      return -1;
    }
    gettimeofday(&ptu_executed_time, NULL);
    ptu_executed_time_stamp = (uint64_t)(ptu_executed_time.tv_sec) * 1000000000 + (uint64_t)(ptu_executed_time.tv_usec)
        * 1000 + time_stamp_offset;
    usleep(20000);
    bool isScanLatterThanPtu = false;
    while (!isScanLatterThanPtu)
    {
      laser.serviceScan(laser_scan);
      if (ptu_executed_time_stamp < laser_scan.self_time_stamp)
      {
        isScanLatterThanPtu = true;
        scans.push_back(laser_scan);
      }
    }
  }
  laser.stopScanning();
  cloud->height = scans.size() * 2;
  cloud->width = scans[0].ranges.size() / 2;
  cloud->points.resize(cloud->height * cloud->width);

  for (int i = 0; i < static_cast<int>(scans.size()); i++)
  {
    double cos_1 = cos(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double sin_1 = sin(-(position + i * offset) * FULL_STEP_RES * M_PI / 180);
    double cos_2 = -cos_1;
    double sin_2 = -sin_1;
    for (int j = min_step; j < afrt; j++)
    { 
      if (scans[i].ranges[j] > 0.1 && scans[i].ranges[j] < 30)
      {
        cloud->points[i * cloud->width + afrt - j - 1].x = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * cos_1;
        cloud->points[i * cloud->width + afrt - j - 1].y = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180) * sin_1;
        cloud->points[i * cloud->width + afrt - j - 1].z = scans[i].ranges[j] * cos((afrt - j) * 0.25 * M_PI / 180);
        PCL_DEBUG("scans[%d].intensities[%d]: %f\n",i,j, scans[i].intensities[j]);
        cloud->points[i * cloud->width + afrt - j - 1].range = scans[i].ranges[j];
      }
      else 
      {
	cloud->points[i * cloud->width + afrt - j - 1].x = 0.0;
	cloud->points[i * cloud->width + afrt - j - 1].y = 0.0;
        cloud->points[i * cloud->width + afrt - j - 1].z = 0.0;
        cloud->points[i * cloud->width + afrt - j - 1].range = 0.0;
      }
    }
    for (int j = afrt + 1; j <= max_step; j++)
    {
      if (scans[i].ranges[j] > 0.1 && scans[i].ranges[j] < 30)
      {
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * cos_2;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = scans[i].ranges[j] * sin((j - afrt) * 0.25 * M_PI / 180) * sin_2;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = scans[i].ranges[j] * cos((j - afrt) * 0.25 * M_PI / 180);
        PCL_DEBUG("scans[%d].intensities[%d]: %f\n",i,j, scans[i].intensities[j]);
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].range = scans[i].ranges[j];
      }
      else
      {
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = 0.0;
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = 0.0;
        cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = 0.0;
	cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].range = 0.0;
      }
    }
  }
  
  PCL_INFO("3D scan has been acquired!\n");
  return 0;
}


int PtuLaser::get3DScan(PointCloudXYZ::Ptr &cloud, 
      const double step_angle,
      const double pan_range,
      const bool clockwise)
{
  scans.clear();
  signed short int offset = static_cast<signed short int> (step_angle / FULL_STEP_RES);
  if (clockwise == false)
  {
    offset = -offset;
  }
  int afrt = laser.afrt();
  long long int time_stamp_offset = laser.getHokuyoClockOffset(10);
  laser.laserOn();
  laser.requestScans(false, min_step, max_step, 1, 0, 0, -1);

  int position = ptu::get_current(PAN, POSITION);
  std::cout << "step number:" << abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset)))) << std::endl;
  for (int step = 0; step < abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset))))+1; step++)
  {
    if ((status = ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, RELATIVE)) == TRUE)
    {
      return -1;
    }
    if ((status = ptu::await_completion()) != PTU_OK)
    {
      return -1;
    }
    gettimeofday(&ptu_executed_time, NULL);
    ptu_executed_time_stamp = (uint64_t)(ptu_executed_time.tv_sec) * 1000000000 + (uint64_t)(ptu_executed_time.tv_usec)
        * 1000 + time_stamp_offset;
    usleep(20000);
    bool isScanLatterThanPtu = false;
    while (!isScanLatterThanPtu)
    {
      //laser.pollScan(laser_scan,false, min_step, max_step, 1, -1);
      laser.serviceScan(laser_scan);
      if (ptu_executed_time_stamp < laser_scan.self_time_stamp)
      {
        isScanLatterThanPtu = true;
        scans.push_back(laser_scan);
      }
    }
  }
  laser.stopScanning();
  cloud->height = scans.size() * 2;
  cloud->width = scans[0].ranges.size() / 2;
  cloud->points.resize(cloud->height * cloud->width);
  for (size_t i = 0; i < scans.size(); i++)
  {
    double cos_1 = cos(-(position + static_cast<int> (i) * offset) * FULL_STEP_RES * M_PI / 180);
    double sin_1 = sin(-(position + static_cast<int> (i) * offset) * FULL_STEP_RES * M_PI / 180);
    double cos_2 = -cos_1;
    double sin_2 = -sin_1;
    for (int j = min_step; j < afrt; j++)
    {
      cloud->points[i * cloud->width + afrt - j - 1].x = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180)
          * cos_1;
      cloud->points[i * cloud->width + afrt - j - 1].y = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180)
          * sin_1;
      cloud->points[i * cloud->width + afrt - j - 1].z = scans[i].ranges[j] * cos((afrt - j) * 0.25 * M_PI / 180);
    }
    for (int j = afrt + 1; j <= max_step; j++)
    {
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].x = scans[i].ranges[j] * sin((j - afrt) * 0.25
          * M_PI / 180) * cos_2;
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].y = scans[i].ranges[j] * sin((j - afrt) * 0.25
          * M_PI / 180) * sin_2;
      cloud->points[(i + scans.size()) * cloud->width + j - 1 - afrt].z = scans[i].ranges[j] * cos((j - afrt) * 0.25
          * M_PI / 180);
    }
  }
  return 0;
}

int PtuLaser::scanDellBoxes(PointCloudXYZ::Ptr &cloud,
                            const double step_angle,
                            const double pan_range,
                            const bool clockwise)
{
  scans.clear();
  signed short int offset = static_cast<signed short int> (step_angle / FULL_STEP_RES);
  if (clockwise == false)
  {
    offset = -offset;
  }
  int afrt = laser.afrt();
  long long int time_stamp_offset = laser.getHokuyoClockOffset(10);
  laser.laserOn();
  laser.requestScans(false, min_step, max_step, 1, 0, 0, -1);

  int position = ptu::get_current(PAN, POSITION);
  std::cout << "step number:" << abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset)))) << std::endl;
  for (int step = 0; step < abs(static_cast<int> ((pan_range / M_PI) * 180 / (FULL_STEP_RES
      * static_cast<double > (offset))))+1; step++)
  {
    if ((status = ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, RELATIVE)) == TRUE)
    {
      return -1;
    }
    if ((status = ptu::await_completion()) != PTU_OK)
    {
      return -1;
    }
    gettimeofday(&ptu_executed_time, NULL);
    ptu_executed_time_stamp = (uint64_t)(ptu_executed_time.tv_sec) * 1000000000 + (uint64_t)(ptu_executed_time.tv_usec)
        * 1000 + time_stamp_offset;
    usleep(20000);
    bool isScanLatterThanPtu = false;
    while (!isScanLatterThanPtu)
    {
      //laser.pollScan(laser_scan,false, min_step, max_step, 1, -1);
      laser.serviceScan(laser_scan);
      if (ptu_executed_time_stamp < laser_scan.self_time_stamp)
      {
        isScanLatterThanPtu = true;
        scans.push_back(laser_scan);
      }
    }
  }
  laser.stopScanning();
  cloud->height = scans.size();
  cloud->width = scans[0].ranges.size() / 2;
  cloud->points.resize(cloud->height * cloud->width);
  for (size_t i = 0; i < scans.size(); i++)
  {
    double cos_1 = cos(-(position + static_cast<int> (i) * offset) * FULL_STEP_RES * M_PI / 180);
    double sin_1 = sin(-(position + static_cast<int> (i) * offset) * FULL_STEP_RES * M_PI / 180);
    double cos_2 = -cos_1;
    double sin_2 = -sin_1;
    for (int j = min_step; j < afrt; j++)
    {
      cloud->points[i * cloud->width + afrt - j - 1].x = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180)
          * cos_1;
      cloud->points[i * cloud->width + afrt - j - 1].y = scans[i].ranges[j] * sin((afrt - j) * 0.25 * M_PI / 180)
          * sin_1;
      cloud->points[i * cloud->width + afrt - j - 1].z = scans[i].ranges[j] * cos((afrt - j) * 0.25 * M_PI / 180);
    }
  }

  offset = -250;
  ptu::set_desired(PAN, POSITION, (ptu::PTU_PARM_PTR *)&offset, ABSOLUTE);
  return 0;
}
