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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <boost/timer.hpp>
#include "fftw_correlate/fftw_correlate.h"
#include "application_options_manager/application_options_manager.h"
#include "common/planar_patch.h"

int
main (int argc, char **argv)
{
  using namespace tams;
  std::ifstream file_in;
  std::ofstream of;
  ApplicationOptionsManager amgr;
  if (!amgr.readOptions(argc, argv)) return -1;

  std::string segments_prefix = amgr.app_options_.segments_dir;
  std::string pcd_prefix = amgr.app_options_.unorganized_pcd_dir + amgr.app_options_.input_prefix;

  int bandwidth = 128;
  int egi_size = 4 * bandwidth * bandwidth;
  float *egi = new float[egi_size];
  float egi_resolution_theta = M_PI / (bandwidth * 2);
  float egi_resolution_phi = M_PI / bandwidth;
  float theta, phi;

  FFTWCorrelate fc;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointXYZ> points;
  boost::timer t;

  fc.setBandWidth (128, 128, 127);
  t.restart();
  fc.initialize();

  for (size_t scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    memset(egi, 0.0, egi_size * sizeof(float));
    char buf[4];
    sprintf(buf,"%03d", scan_index);
    std::string map_file = pcd_prefix + std::string (buf) + ".pcd";
    std::string map_segments = segments_prefix + "segments" + std::string(buf);

    sprintf(buf,"%03d", scan_index + 1);
    std::string data_file = pcd_prefix + std::string (buf) + ".pcd";
    std::string data_segments = segments_prefix + "segments" + std::string(buf);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_file, *map_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", map_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              map_cloud->points.size(), map_cloud->width, map_cloud->height, map_file.c_str());

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (data_file, *data_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", data_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              data_cloud->points.size(), data_cloud->width, data_cloud->height, data_file.c_str());


    fc.setMap(map_cloud);
    fc.setData(data_cloud);

    //fc.mapConstellationImage(map_segments);
    //fc.dataConstellationImage(data_segments);

    fc.egiMap(false, false, false);
    fc.egiData(false, false, false);

    fc.softFFTWCorrelateReal ();
    fc.visualize();
  }
}
