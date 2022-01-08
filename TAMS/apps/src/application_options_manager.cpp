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

//STL
#include <iostream>
#include <fstream>
#include <string>
//boost
#include <boost/filesystem.hpp>
//tams
#include "application_options_manager/application_options_manager.h"
#include "region_growing_segmentation/region_growing_segmentation_parameters.h"


namespace po = boost::program_options;
namespace tams
{
  ApplicationOptionsManager::ApplicationOptionsManager():
      all_opts_desc_ ("All options"), visible_opts_desc_ ("General options"),
     sensor_opts_desc_ ("Sensor options"),
      seg_opts_desc_ ("General Segmentation options"),
      octree_seg_opts_desc_ ("General Octree Segmentation options"),
      output_opts_desc_ ("output options"), input_opts_desc_ ("input options"),
      registration_desc_ ("registration options"), cmd_line_pd_opts_desc_ (),
    vm_(), sensor_params_ (), seg_params_ (), app_options_ (), app_states_ ()
  {

  }
  void ApplicationOptionsManager::defineProgramOptions(void)
  {
    visible_opts_desc_.add_options()
      ("help,h", "produces help message")
      ("pcd-file", po::value<std::string>(&(app_options_.pcd_file)), "name of a pcd file to be processed")
      ("config-file", po::value<std::string>(&(app_options_.config_file)), "name of segmentation config file");

    sensor_opts_desc_.add_options()
        ("sensor.vertical-resolution", po::value<double>(&(sensor_params_.vertical_resolution)), "vertical resolution of the range sensor")
        ("sensor.horizontal-resolution", po::value<double>(&(sensor_params_.horizontal_resolution)), "horizontal resolution of the range sensor")
        ("sensor.max-range", po::value<double>(&(sensor_params_.max_range)), "the sensor's detection range")
        ("sensor.poly-standard-deviation.a0", po::value<double>(&(sensor_params_.polynomial_noise_a0)), "the polynomial noisy model")
        ("sensor.poly-standard-deviation.a1", po::value<double>(&(sensor_params_.polynomial_noise_a1)), "the polynomial noisy model")
        ("sensor.poly-standard-deviation.a2", po::value<double>(&(sensor_params_.polynomial_noise_a2)), "the polynomial noisy model");

    seg_opts_desc_.add_options()
      ("seg.sliding-window-size", po::value<int>(&(seg_params_.sliding_window_size)), "sliding window size")
      ("seg.min-segment-size", po::value<int>(&(seg_params_.min_segment_size)), "minimum segment size")
      ("seg.max-neighbor-dist", po::value<double>(&(seg_params_.max_neighbor_dis)),
          "if distance between two points is less than this threshold the points are considered to be neighbors")
      ("seg.max-segment-mse", po::value<double> (&(seg_params_.max_segment_mse)),
          "max allowed mean square error for the optimal plane")
      ("seg.max-local-mse", po::value<double> (&(seg_params_.max_local_mse)),
          "the angle difference check will be applied if local mse is smaller than this threshold")
      ("seg.max-seed-mse", po::value<double> (&(seg_params_.max_seed_mse)),
          "the point can be selected as a seed if its local mse is less than this value")
      ("seg.nearest-neighbor-size", po::value<int> (&(seg_params_.nearest_neighbor_size)),
          "8 or 24 nearest neighbor will be investigated")
      ("seg.max-point-dist", po::value<double>(&(seg_params_.max_point2plane_dis)),
          "max allowed distance of point from the optimal plane")
      ("seg.max-angle-diff-deg", po::value<double>(&(seg_params_.max_angle_difference)),
          "max allowed angle between local plane and optimal plane");

    octree_seg_opts_desc_.add_options()
      ("octree-seg.max-neighbor-dis", po::value<double>(&(octree_seg_params_.max_neighbor_dis)), "the point will be investigated nearer than this dis")
      ("octree-seg.max-point2plane-dis", po::value<double>(&(octree_seg_params_.max_point2plane_dis)), "test")
      ("octree-seg.max-angle-difference", po::value<double>(&(octree_seg_params_.max_angle_difference)), "test")
      ("octree-seg.max-segment-mse", po::value<double>(&(octree_seg_params_.max_segment_mse)),"test")
      ("octree-seg.max-local-mse", po::value<double>(&(octree_seg_params_.max_local_mse)), "test")
      ("octree-seg.min-segment-size", po::value<int>(&(octree_seg_params_.min_segment_size)), "test")
      ("octree-seg.sliding-sphere-size", po::value<int>(&(octree_seg_params_.sliding_sphere_size)), "test")
      ("octree-seg.nearest-neighbor-size", po::value<int>(&(octree_seg_params_.nearest_neighbor_size)), "test")
      ("octree-seg.max-seed-mse", po::value<double>(&(octree_seg_params_.max_seed_mse)), "test")
      ("octree-seg.downsampling", po::value<bool>(&(octree_seg_params_.downsampling)), "test")
      ("octree-seg.show-filtered-cloud", po::value<bool>(&(octree_seg_params_.show_filtered_cloud)), "test")
      ("octree-seg.downsampling-leafsize", po::value<double>(&(octree_seg_params_.downsampling_leafsize)), "test")
      ("octree-seg.osr-mean-k", po::value<int>(&(octree_seg_params_.osr_mean_k)), "test")
      ("octree-seg.osr-std-dev-mul-thresh", po::value<double>(&(octree_seg_params_.osr_StddevMulThresh)), "test")
      ;

    input_opts_desc_.add_options()
      ("input.organized-pcd-dir", po::value<std::string>(&(app_options_.organized_pcd_dir)),"directory where organized point clouds are")
      ("input.unorganized-pcd-dir", po::value<std::string>(&(app_options_.unorganized_pcd_dir)),"directory where unorganized point clouds are")
      ("input.segments-dir", po::value<std::string>(&(app_options_.segments_dir)),"directory where input files are")
      ("input.pcd-prefix", po::value<std::string>(&(app_options_.input_prefix)), "prefix for input files")
      ("input.first-index", po::value<int>(&(app_options_.first_index)), "first scan number")
      ("input.last-index", po::value<int>(&(app_options_.last_index)), "last scan number");

    output_opts_desc_.add_options()
      ("output.directory", po::value<string>(&(app_options_.output_dir)), "directory where output files should be stored")
      ("output.suffix", po::value<string>(&(app_options_.output_suffix)), "suffix to be added to the filenames of output files")
      ("output.color-segments", po::value<bool>(&(app_options_.color_segments)), "wheter to display colored segments");

    registration_desc_.add_options()
      ("registration.visualization", po::value<bool>(&(registration_params_.visualization)),"whether to visualize the registation result")
      ("registration.min-area", po::value<double>(&(registration_params_.min_area)), "planar patches with area bigger than this threshold will be used in the registration")
      ("registration.merge-angle", po::value<double>(&(registration_params_.merge_angle)), "planar patches to be merged for avoiding one-to-many correspondences")
      ("registration.merge-dis", po::value<double>(&(registration_params_.merge_dis)), "planar patches to be merged for avoiding one-to-many correspondences")
      ("registration.max-area-diff", po::value<double>(&(registration_params_.max_area_diff)), "planar patches are marked as potential correspondence if their area difference is smaller than this value")
      ("registration.unparallel-min-angle", po::value<double>(&(registration_params_.unparallel_min_angle)), "two planar patches are marked as unparallel if their angle is bigger than this threshold")
      ("registration.unparallel-max-angle", po::value<double>(&(registration_params_.unparallel_max_angle)), "two planar patches are marked as unparallel if their angle is smaller than this threshold")
      ("registration.side-rotation-ability", po::value<double>(&(registration_params_.side_rotation_ability)), "the biggest rotation angle according to the robot's kinematic model")
      ("registration.max-translation-norm", po::value<double>(&(registration_params_.max_translation_norm)), "the biggest distance between two sensor poses")
      ("registration.max-angle-diff", po::value<double>(&(registration_params_.max_angle_diff)), "the threshold for rotation consistent")
      ("registration.max-bias-diff", po::value<double>(&(registration_params_.max_bias_diff)), "the threshold for transformation consistent");
    visible_opts_desc_.add(seg_opts_desc_);
    visible_opts_desc_.add(sensor_opts_desc_);
    visible_opts_desc_.add(octree_seg_opts_desc_);
    visible_opts_desc_.add(output_opts_desc_);
    visible_opts_desc_.add(input_opts_desc_);
    visible_opts_desc_.add(registration_desc_);
    cmd_line_pd_opts_desc_.add("config-file", 1);
    cmd_line_pd_opts_desc_.add("pcd-file", 2);
    all_opts_desc_.add(visible_opts_desc_);
  }

  bool
  ApplicationOptionsManager::readOptions (int argc, char* argv[])
  {
    bool result = true;
    defineProgramOptions ();

    int status = parseOptions (argc, argv);
    result = (status == 0);

    return result;
  }

    void
  ApplicationOptionsManager::printHelpMessage (std::ostream& os)
  {
    os << "Usage: " << std::endl;
    os << "\t planar_segmentation [options] [config.ini] [example.pcl]" << std::endl;
    os << "\t planar_segmentation [options] [--config-file=config.ini] [--pcl-file=example.pcl]" << std::endl;
    os << std::endl;
    os << visible_opts_desc_ << std::endl;
  }

  int
  ApplicationOptionsManager::parseOptions (int argc, char* argv[])
  {
    po::store (po::command_line_parser (argc, argv).options (all_opts_desc_).positional (cmd_line_pd_opts_desc_).run (),
               vm_);
    if (vm_.count ("config-file"))
    {
      std::cout << "Using config file: " << vm_["config-file"].as<std::string>() << std::endl;
      std::ifstream fin (vm_["config-file"].as<std::string> ().c_str ());
      if (!fin.is_open ())
      {
        std::cerr << "Can't open specified config file \"" << vm_["config-file"].as<std::string> () << "\"" << std::endl;
      }
      else
      {
        po::store (po::parse_config_file (fin, all_opts_desc_, true), vm_);
      }
    }
    po::notify(vm_);
    if (vm_.count ("help"))
    {
      printHelpMessage(std::cout);
      return 1;
    }

    if (!vm_.count ("pcd-file"))
    {
      std::cerr << "pcd file wasn't specified aborting!" << std::endl;
      printHelpMessage(std::cout);
      return 2;
    }

    // Create output folder if it does not exist
    if (vm_.count ("output.directory"))
    {
      boost::filesystem::path odir (vm_["output.directory"].as<std::string> ());
      if (boost::filesystem::exists (odir))
      {
        std::cout << "The output folder " << odir.string () << " exists. Files will be overwritten." << std::endl;
      }
      else
      {
        std::cout << "The output folder " << odir.string () << " does not exist. It will be created." << std::endl;
        if (!boost::filesystem::create_directory (odir))
        {
          std::cerr << "Output folder " << odir.string () << " could not be created. Check permissions." << std::endl;
        }
      }
    }

    return 0;
  }

  void
  ApplicationOptionsManager::outputOptions (std::ostream &os)
  {
    os << "#Application options: " << std::endl;
    os << "\t pcd-file = " << app_options_.pcd_file << endl;
    os << "\t config-file = " << (app_options_.config_file == "" ? "not specified" : app_options_.config_file) << std::endl;
    os << "#General segmentation options: " << std::endl;
    os << "\t segmentation.min-segment-size = " << seg_params_.min_segment_size << std::endl;
    os << "#Region growing options: " << std::endl;
    os << "\t seg.reg-grower.max-neighbor-dist = " << seg_params_.max_neighbor_dis << std::endl;
    os << "\t seg.reg-grower.max-plane-mse = " <<  seg_params_.max_segment_mse << std::endl;
    os << "\t seg.reg-grower.max-point-dist = " << seg_params_.max_point2plane_dis << std::endl;
    os << "\t seg.reg-grower.max-angle-diff-deg = " << seg_params_.max_angle_difference << std::endl;
    os << std::endl;
    os << "#Output options" << std::endl;
    os << "\t output.directory = " << app_options_.output_dir << std::endl;
    os << "\t output.suffix = " << app_options_.output_suffix << std::endl;
    os << std::endl;
  }

}
