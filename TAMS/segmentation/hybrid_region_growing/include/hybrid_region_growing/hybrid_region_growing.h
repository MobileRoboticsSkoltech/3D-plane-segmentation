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

#ifndef HYBRID_REGION_GROWING_SEGMENTATION_H_
#define HYBRID_REGION_GROWING_SEGMENTATION_H_
//pcl header files
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "Eigen/Core"
#include "pcl/visualization/pcl_visualizer.h"
#include "pcl/visualization/point_cloud_handlers.h"
//system header files
#include <sys/time.h>
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <math.h>
#include <fstream>
#include <utility>
//self developed header files
#include "common/planar_patch.h"
#include "common/subwindow.h"
#include "common/rgb.h"
#include "hybrid_region_growing/hybrid_region_growing_parameters.h"

using namespace Eigen;
namespace tams{

struct SimpleSubwindow
{
  int index;
  double mse;
  SimpleSubwindow () :
    index (0), mse (0)
  {
  }
  ~SimpleSubwindow ()
  {
  }
  bool
  operator< (const SimpleSubwindow& rhs) const
  {
    return mse < rhs.mse;
  }
};

class HybridRGSegmentation
{
  public:
  /**@b Empty construction.*/
  HybridRGSegmentation():
    cloud_ (new pcl::PointCloud<pcl::PointXYZ>), size_ (0), height_ (0), width_ (0),
    subwindows_height_ (0), subwindows_width_ (0), valid_ (NULL), visited_ (NULL),
    added_to_region_ (NULL), isPlanar_ (NULL), local_mse_ (NULL), valid_points_ (0),
    planar_subwindows_cnt_ (0), badpoints_num_ (0), valid_indices_ (NULL)
  {

  }

  /** @b Providing the input cloud, the points will be represented by Eigen::Vector3d for efficient linear algebra.*/
  void
  setInput (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    cloud_ = cloud;
  }
  /**
   * @b Construct the subwindows, i.e., compute their attributes.
   * @param[in] depth it determines the size of each subwindow, size = (2 * depth + 1) * (2 * depth +1)
   */
  void
  subwindows(int depth);

  void
  subwindows(int side_length, bool with_indices);

  /**
   * @b Set thresholds for the region growing.
   * @param parameters
   */
  void
  setparameters (const HybridRGSegmentationParameters parameters)
  {
    parameters_ = parameters;
  }

  /**
   * @b When one subwindow is added to the segment,
   * its neighbors should be tested whether to be pushed into the neighbors list.
   * @param[in] pos the position off added subwindos.
   */
  void
  investigate8Neighbors (const int pos);

  /** \brief Segment the input cloud into big planar patches.*/
  void
  applySegmentation();

  /** \brief Set random color to the detected big planar patches.
   * The colored planar patches will be put into cloud output->
   * @param output cloud with colored planar patches
   */
  void
  colorEncoding (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,
                 bool project2plane);

  void
  preprocessing();

  void
  savetimes ();

  /** @b Empty destructor. */
  ~HybridRGSegmentation (){}
  public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    int size_;
    int height_;
    int width_;
    int subwindows_height_;
    int subwindows_width_;
    std::vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points_;
    PlanarSegment::StdVector planar_patches_;
    Subwindow::StdVector subwindows_;
    //std::vector<int> neighbors_;
    std::deque<int> neighbors_;
    std::vector<int> remained_points_;
    bool *valid_;
    bool *visited_;
    bool *added_to_region_;
    bool *isPlanar_;
    double *local_mse_;
    std::vector<SimpleSubwindow> sorted_subwindows_;
    int valid_points_;
    int planar_subwindows_cnt_;
    int badpoints_num_;
    int *valid_indices_;
    std::vector<timeval> times;
    timeval tmp_time;
    HybridRGSegmentationParameters parameters_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
#endif
