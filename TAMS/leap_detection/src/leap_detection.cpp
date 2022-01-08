/*
 * Software License Agreement (BSD License)
 *
 *  Technical Aspects of Multimodal Systems (TAMS) - http://tams-www.informatik.uni-hamburg.de/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TAMS, nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Junhao Xiao
 * Email  : junhao.xiao@ieee.org, xiao@informatik.uni-hamburg.de
 */

#include "leap_detection/leap_detection.h"
#include <pcl/io/pcd_io.h>
int
main (int argc, char **argv)
{
  LeapDetection leap_detection;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string pcd_file(argv[1]);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
            cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());
  double threshold = 30 * 30;
  for (size_t i = 0; i < cloud->size(); i++)
  {
    if (cloud->points[i].z < 0 && (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y) < 0.64)
    {
      cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = 0;
    }
    if (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y + cloud->points[i].z * cloud->points[i].z > threshold)
    {
      cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = 0;
    }
  }

  leap_detection.setInputCloud(cloud);
  PCL_INFO ("set input finished!\n");
  leap_detection.getUognzdCloud();
  PCL_INFO ("get unorganized cloud finished!\n");
  leap_detection.leapCandidates();
  PCL_INFO ("detect leap candidates finished!\n");
  leap_detection.detectLeaps();
  PCL_INFO ("leap founded!\n");
  return 0;
}
