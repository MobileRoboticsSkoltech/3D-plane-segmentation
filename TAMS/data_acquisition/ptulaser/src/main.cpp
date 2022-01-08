#include "ptulaser/ptulaser.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#define KEYCODE_P 0x70
#define KEYCODE_P_CAP 0x50

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr graycloud(new pcl::PointCloud<pcl::PointXYZI>);
  IplImage * range;
  IplImage * intensity;
  double step_angle = 0.5;
  PtuLaser ptulaser;
  ptulaser.LaserConf();
  ptulaser.PtuConf();

  std::string prefix = "../data/";
  std::string pcd_file;
  std::string range_file;
  std::string intensity_file;
  int index;
  if (argc == 1)
    index = 0;
  if (argc == 2)
  {
    char *buf;
    buf = argv[1];
    index = atoi(buf);
  }

  char buf[4];
  sprintf (buf, "%03d", index);
  pcd_file = prefix + "scan" + std::string (buf) + ".pcd";
  range_file = prefix + "range" + std::string (buf) + ".png";
  intensity_file = prefix + "intensity" + std::string (buf) + ".png";

  char c;
  bool scan = false;
  bool has_intensity = false;
  //clear the memory
  signal(SIGINT,quit);
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'p' to actuate the ptu and laser to get a cloud with XYZ and intensity ");
  puts("Use 'p' + 'Shift' to actuate the ptu and laser to get a cloud with XYZ");

  for(;;)
  {
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    switch(c)
    {
    //take a ptu + hokuyo laser scan
    case KEYCODE_P:
      scan = true;
      has_intensity = true;
      break;
    case KEYCODE_P_CAP:
      scan = true;
      has_intensity = false;
      break;
    default:
      scan = false;
      break;
    }
    if (scan == true)
    {
      if (has_intensity == true)
      {
        std::cout << "scan with intensity" << std::endl;
        ptulaser.get3DScanWithIntensity(graycloud,range,intensity,step_angle,M_PI);
        pcl::io::savePCDFileASCII(pcd_file, *graycloud);
        double  max_intensity, min_intensity;
        double  max_range, min_range;
        cvMinMaxLoc(range,&min_range,&max_range);
        cvMinMaxLoc(intensity,&min_intensity,&max_intensity);
        cvConvertScale(range,range,255/(max_range-min_range),-min_range);
        cvConvertScale(intensity,intensity,255/(max_intensity-min_intensity),-min_intensity);
        char *str = const_cast<char*>(range_file.c_str());
        cvSaveImage(str, range);
        str = const_cast<char*>(intensity_file.c_str());
        cvSaveImage(str, intensity);
        index ++;
      }
      if (has_intensity == false)
      {
        //ptulaser.get3DScan(cloud,step_angle,M_PI);
        ptulaser.scanDellBoxes(cloud, 0.5, 20.0 * M_PI / 180);
        pcl::io::savePCDFileBinary(pcd_file, *cloud);
        index ++;
      }
      scan = false;
      sprintf (buf, "%03d", index);
      pcd_file = prefix + "scan" + std::string (buf) + ".pcd";
      range_file = prefix + "range" + std::string (buf) + ".png";
      intensity_file = prefix + "intensity" + std::string (buf) + ".png";
    }
  }

  cvReleaseImage(&range);
  cvReleaseImage(&intensity);
  return 0;
}
