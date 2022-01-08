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

#include "region_growing_segmentation/region_growing_segmentation.h"
#include "region_growing_segmentation/impl/region_growing_segmentation.hpp"
#include "application_options_manager/application_options_manager.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <boost/timer.hpp>
#include <fstream>

int
main (int argc, char **argv)
{
  using namespace tams;
  std::ofstream of;
  ApplicationOptionsManager amgr;
  if (!amgr.readOptions (argc, argv))
    return -1;
  RGSegmentation<pcl::PointXYZ> segmenter;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string prefix = amgr.app_options_.organized_pcd_dir + amgr.app_options_.input_prefix;

  struct timeval tpstart,tpend;
  double timeuse;
  std::ofstream time_output;
  time_output.open("indoorHokuyo.txt");
  
  for (int scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    //std::string pcd_file = amgr.app_options_.pcd_file;
    std::string segments_file = amgr.app_options_.output_dir + "/segments" + std::string (buf);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str ());

    segmenter.setParameters (amgr.seg_params_);
    PlanarSegment::StdVector segments;
    segmenter.setInputCloud (cloud);
    gettimeofday(&tpstart,NULL);
    segmenter.segmentation (output);
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "segmentation time: " << timeuse << std::endl;
    segmenter.computeSegmentsArea ();
    segmenter.getSegments(segments);
    if (amgr.app_options_.color_segments)
    {
      segmenter.colorEncoding (output);
      pcl::visualization::PCLVisualizer viewer ("Detected planes with Pseudo-color");
      viewer.setBackgroundColor (1.0, 1.0, 1.0);
      viewer.addPointCloud (output, "point cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "point cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point cloud");
      for (size_t i = 0; i < segments.size (); i++)
      {
        std::cerr << segments[i].area << std::endl;
        if (segments[i].area < 0.2)
          continue;
        std::ostringstream area, area_full;
        area << setiosflags(ios::fixed) << setprecision(2) << segments[i].area;
        area_full << segments[i].area;
        pcl::PointXYZ position;
        position.x = segments[i].mass_center (0);
        position.y = segments[i].mass_center (1);
        position.z = segments[i].mass_center (2);
        viewer.addText3D (area.str (), position, 0.4, 0.0, 0.0, 0.0, area_full.str (), 0);
      }
      viewer.spin ();
    }
  }
  return (0);
}
