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

#ifndef LEAP_DETECTION_H_
#define LEAP_DETECTION_H_
#include <boost/shared_ptr.hpp>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class LeapDetection
{

public:
  LeapDetection():
    ognzd_input_ (new pcl::PointCloud<pcl::PointXYZ>),
    uognzd_input_ (new pcl::PointCloud<pcl::PointXYZ>),
    grid_size_ (0), height_ (0), width_ (0)
  {}
  ~LeapDetection()
  {
    if (valid_ != NULL)
      delete [] valid_;
    if (leap_candidates_ != NULL)
      delete [] leap_candidates_;
  }
  void leapCandidates();
  void detectLeaps();
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void getUognzdCloud();
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr ognzd_input_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr uognzd_input_;
  std::vector<int> leap_indices_;
  std::vector<int> ognzd_indices_in_uognzd_cloud_;
  std::vector<int> uognzd_indices_in_ognzd_cloud_;
  int grid_size_;
  int height_;
  int width_;
  int *leap_candidates_;
  int *leaps_;
  int *valid_;
};
#endif
