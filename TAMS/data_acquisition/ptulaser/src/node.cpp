#include "ptulaser/ptulaser.h"
#include "opencv2/highgui/highgui_c.h"
#include <stdio.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZI>::Ptr graycloud(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<tamrot::PointXYZIWithRange>::Ptr cloudWithRangeAndIntensity(new pcl::PointCloud<tamrot::PointXYZIWithRange>);
pcl::PointCloud<pcl::PointWithRange>::Ptr cloudWithRange (new pcl::PointCloud<pcl::PointWithRange>);

IplImage * depth;
IplImage * intensity;
PtuLaser ptulaser;
int scanNum;
std::string prefix;
std::string fileName;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void execute3DScan_callback(const msgs::execute3DScan &msg);

int main(int argc, char **argv)
{
  ptulaser.LaserConf();
  ptulaser.PtuConf();
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ptulaser_node");
  prefix = "../data/";
  if (argc == 2)
  {
    char *buf ;
    buf = argv[1];
    scanNum = atoi(buf);
  }
  else 
  {
    scanNum = 0;
  }
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("execute3DScan", 1, execute3DScan_callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

void execute3DScan_callback(const msgs::execute3DScan &msg)
{
  ROS_DEBUG_STREAM("execute3DScan callback"<<std::endl);
  doublestep_angle = msg.step_angle; 
  //doublestep_angle = 60;
  bool hasIntensity = msg.hasIntensity;
  doublepan_range = msg.pan_range;
  bool clockwise = msg.clockwise;

  if (hasIntensity == true)
  {
    ptulaser.get3DScanWithIntensity(graycloud, depth, intensity, step_angle, pan_range, clockwise);
    ptulaser.get3DScanWithRangeAndIntensity(cloudWithRangeAndIntensity, 
                                            depth, 
                                            intensity, 
                                            step_angle, 
                                            pan_range, 
                                            clockwise);
    char *buf = new char[8];
    fileName = prefix + std::string("depth");
    sprintf(buf,"%04d.png",scanNum);
    fileName += std::string(buf);
    char *str = const_cast<char*>(fileName.c_str());
    //std::cout<<str<<" "<<strlen(str)<<std::endl;
    //cvSaveImage("../data/depth0000.png",depth);
    cvSaveImage(str,depth);
    fileName = prefix + std::string("intensity");
    fileName += std::string(buf);
    str = const_cast<char*>(fileName.c_str());
    cvSaveImage(str,intensity);
    fileName = prefix + std::string("xyzdi");
    
    sprintf(buf,"%04d.pcd",scanNum);
    fileName += std::string(buf);
    
    //pcl::io::savePCDFileASCII(fileName.c_str(), *graycloud);
    pcl::io::savePCDFileASCII(fileName.c_str(), *cloudWithRangeAndIntensity);
    cvReleaseImage(&depth);
    cvReleaseImage(&intensity);
    scanNum++;
    delete buf;
  }
  else //(intensity = false)
  {
    //ptulaser.get3DScan(cloud, step_angle, pan_range, clockwise);
    ptulaser.get3DScanWithRange (cloudWithRange,
				 step_angle,
				 pan_range,
				 clockwise);
    char *buf = new char[8];
    fileName = prefix + std::string("xyzWithRange");
    sprintf(buf,"%4d.pcd",scanNum);
    pcl::io::savePCDFileASCII(fileName.c_str(), *cloud);
    scanNum++;
    delete buf;
  }

}
