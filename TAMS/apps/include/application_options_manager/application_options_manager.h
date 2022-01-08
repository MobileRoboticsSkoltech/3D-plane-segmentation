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


#ifndef APPS_OPTIONS_MANAGER_H_
#define APPS_OPTIONS_MANAGER_H_

//boost
#include <boost/program_options.hpp>
//tams
#include "common/sensor_parameters.h"
#include "region_growing_segmentation/region_growing_segmentation_parameters.h"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation_parameters.h"
#include "registration/registration_parameters.h"
namespace po = boost::program_options;
using namespace std;
namespace tams
{
  struct ApplicationOptions
  {
    std::string pcd_file;
    std::string config_file;
    std::string organized_pcd_dir;
    std::string unorganized_pcd_dir;
    std::string segments_dir;
    std::string input_prefix;
    std::string output_dir;
    std::string output_suffix;
    bool color_segments;
    int first_index;
    int last_index;
    ApplicationOptions ()
    {
      output_suffix = output_dir = "";
      pcd_file = config_file = "";
      organized_pcd_dir = unorganized_pcd_dir = "";
      input_prefix = "scan";
      segments_dir = "";
      first_index = last_index = 0;
      color_segments = false;
    }
  };
  struct ApplicationStats
  {
    double segmentation_runtime;
    double area_runtime;
    size_t segments_number;
    size_t badpoints_number;

    ApplicationStats ()
    {
      segmentation_runtime = area_runtime = 0.0;
      segments_number = badpoints_number = 0;
    }
    void
    outputWithNames (std::ostream& os)
    {
      os << "Plane extraction runtime, s: " << segmentation_runtime << std::endl;
      os << "Number of segments: " << segments_number << std::endl;
      os << "Area computation time, s: " << area_runtime << std::endl;
      os << "Number of rejected points: " << badpoints_number << std::endl;
    }
  };
  class ApplicationOptionsManager
  {
  public:
    ApplicationOptionsManager ();
    ~ApplicationOptionsManager () {}
    bool
    readOptions (int argc, char **argv);
    void
    printHelpMessage (std::ostream& os);
    void
    outputOptions (std::ostream &os);
  private:
    void
    defineProgramOptions(void);
    int
    parseOptions (int argc, char* argv[]);
  public:
    po::options_description all_opts_desc_;
    po::options_description visible_opts_desc_;
    po::options_description sensor_opts_desc_;
    po::options_description seg_opts_desc_;
    po::options_description octree_seg_opts_desc_;
    po::options_description output_opts_desc_;
    po::options_description input_opts_desc_;
    po::options_description registration_desc_;
    po::positional_options_description cmd_line_pd_opts_desc_;
    po::variables_map vm_;
    SensorParameters sensor_params_;
    RegionGrowingSegmentationParameters seg_params_;
    OctreeRegionGrowingSegmentationParameters octree_seg_params_;
    RegistrationParameters registration_params_;
    ApplicationOptions app_options_;
    ApplicationStats app_states_;
  };
}
#endif
