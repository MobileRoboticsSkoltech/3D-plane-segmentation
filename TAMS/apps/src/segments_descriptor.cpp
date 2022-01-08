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
 *
 */

#include "segments_descriptor/segments_descriptor.h"
#include "segments_descriptor/impl/segments_descriptor.hpp"
#include "region_growing_segmentation/region_growing_segmentation.h"
#include "region_growing_segmentation/impl/region_growing_segmentation.hpp"
#include "application_options_manager/application_options_manager.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/visualization/point_cloud_handlers.h>
#include <boost/cast.hpp>
int
main(int argc, char **argv)
{
  using namespace tams;
  std::ifstream file_in;
  std::ofstream of;
  ApplicationOptionsManager amgr;
  if (!amgr.readOptions(argc, argv)) return -1;
  //RGSegmentation<pcl::PointXYZ> segmenter;
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //std::string prefix = amgr.app_options_.input_dir + amgr.app_options_.input_prefix;
  SegmentsDescriptor<pcl::PointXYZ> segments_descriptor;
  PlanarSegment::StdVector segments;
  std::string prefix = amgr.app_options_.segments_dir;
  std::vector<std::vector<FeatureBasedOnSegment> > appearances_array;
  std::vector<FeatureBasedOnSegment> appearances;

  for (int i = 0; i < 36; i++)
    std::cout << "(" << cos(i * 10 * M_PI / 180) << "," << sin(i * 10 * M_PI / 180) << ")\n";
  for (int scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    if (scan_index == 601 || scan_index == 920)
      continue;
    char buf[4];
    sprintf(buf,"%03d", scan_index);
    std::string segments_file = prefix + "segments" + std::string(buf);
//    if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) //* load the file
//    {
//      PCL_ERROR ("Couldn't read file %s!\n", fileName.c_str());
//      return (-1);
//    }
//    PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
//              cloud->points.size(), cloud->width, cloud->height, fileName.c_str ());
//    for (size_t i = 0; i < cloud->width; i++)
//      for (size_t j = 0; j < cloud->height; j++)
//      {
//        size_t pos = j * cloud->width + i;
//        if (cloud->points[pos].x * cloud->points[pos].x +
//            cloud->points[pos].y * cloud->points[pos].y +
//            cloud->points[pos].z * cloud->points[pos].z >= 24*24)
//        {
//          cloud->points[pos].x = 0.0;
//          cloud->points[pos].y = 0.0;
//          cloud->points[pos].z = 0.0;
//        }
//      }
//    segmenter.setParameters(amgr.seg_params_);
//    segmenter.setInputCloud(cloud);
//    segmenter.segmentation(output);
//    segmenter.computeSegmentsArea ();
    //segments_descriptor.setInputCloud(cloud);
    file_in.open(segments_file.c_str());
    file_in >> segments_descriptor.volume_ >> segments_descriptor.normalized_volume_ >> segments_descriptor.normalized_volume_;
    int pp_size = 0;
    file_in >> pp_size;
    segments.clear ();
    segments.resize(pp_size);
    int pp_index = 0;
    while (!file_in.eof ())
    {
      file_in >> segments[pp_index].normal(0) >> segments[pp_index].normal(1) >> segments[pp_index].normal(2) >>
          segments[pp_index].bias >> segments[pp_index].mse >> segments[pp_index].area >> segments[pp_index].point_num >>
          segments[pp_index].mass_center(0) >> segments[pp_index].mass_center(1) >> segments[pp_index].mass_center(2);
      pp_index ++;
    }
    file_in.close ();
    PCL_INFO ("Loaded %d planar patches from %s.\n", pp_size, segments_file.c_str());

    segments_descriptor.setSegments(segments);
    segments_descriptor.computeAppearances(appearances);
    if (!appearances.empty())
    {
      appearances_array.push_back(appearances);
    }
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
