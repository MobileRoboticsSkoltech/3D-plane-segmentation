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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <fstream>
void
LeapDetection::leapCandidates()
{
  leap_candidates_ = new int [grid_size_];
  memset(leap_candidates_, 0, grid_size_ * sizeof (int));
  int cnt = 0;
  for (int i = 0; i < height_; i++)
  {
    int index = i * width_;
    int sliding_sum = valid_[index] + valid_[index + 1] + valid_[index + 2] + valid_[index + 3] + valid_[index + 4];
    if (sliding_sum == 5)
      leap_candidates_[index + 2] += 1;
    for (int j = index + 3; j < index + width_ - 2; j++)
    {
      sliding_sum = sliding_sum - valid_[j - 3] + valid_[j + 2];
      if (sliding_sum == 5)
      {
        leap_candidates_[j] += 1;
        cnt ++;
      }
    }
  }
  PCL_INFO ("%d radial leap candidates found!\n", cnt);

  cnt = 0;
  for (int i = 0; i < width_; i++)
  {
    int sliding_sum = valid_[i] + valid_[i + width_] + valid_[i + 2 * width_] + valid_[i + 3 * width_] + valid_[i + 4 * width_];
    if (sliding_sum == 5)
      leap_candidates_[i + 2 * width_] += 1;
    for (int j = 3; j < height_ - 2; j++)
    {
      sliding_sum = sliding_sum - valid_[(j - 3) * width_ + i] + valid_[(j + 2) * width_ + i];
      if (sliding_sum == 5)
      {
        leap_candidates_[j * width_ + i] += 1;
        cnt ++;
      }
    }
  }
  PCL_INFO ("%d tangential_ leap candidates found!\n", cnt);
}

void
LeapDetection::detectLeaps()
{
  leaps_ = new int [grid_size_];
  memset (leaps_, 0, grid_size_ * sizeof (int));
  double x1, x2, y1, y2, z1, z2, cosA, cosB;
  std::ofstream ofA, ofB;
  ofA.open("cosA");
  ofB.open("cosB");
  for (int i = 0; i < grid_size_; i++)
  {
    if (leap_candidates_[i] == 2)
    {
      x1 = ognzd_input_->points[i - 1].x - ognzd_input_->points[i - 2].x;
      y1 = ognzd_input_->points[i - 1].y - ognzd_input_->points[i - 2].y;
      z1 = ognzd_input_->points[i - 1].z - ognzd_input_->points[i - 2].z;
      x2 = ognzd_input_->points[i + 2].x - ognzd_input_->points[i + 1].x;
      y2 = ognzd_input_->points[i + 2].y - ognzd_input_->points[i + 1].y;
      z2 = ognzd_input_->points[i + 2].z - ognzd_input_->points[i + 1].z;
      cosA = (x1 * x2 + y1 * y2 + z1 * z2) / (sqrt(x1 * x1 + y1 * y1 + z1 * z1) * sqrt(x2 * x2 + y2 * y2 + z2 * z2));
      x1 = ognzd_input_->points[i - width_].x - ognzd_input_->points[i - 2 * width_].x;
      y1 = ognzd_input_->points[i - width_].y - ognzd_input_->points[i - 2 * width_].y;
      z1 = ognzd_input_->points[i - width_].z - ognzd_input_->points[i - 2 * width_].z;
      x2 = ognzd_input_->points[i + 2 * width_].x - ognzd_input_->points[i + width_].x;
      y2 = ognzd_input_->points[i + 2 * width_].y - ognzd_input_->points[i + width_].y;
      z2 = ognzd_input_->points[i + 2 * width_].z - ognzd_input_->points[i + width_].z;
      cosB = (x1 * x2 + y1 * y2 + z1 * z2) / (sqrt(x1 * x1 + y1 * y1 + z1 * z1) * sqrt(x2 * x2 + y2 * y2 + z2 * z2));
      ofA << cosA << std::endl;
      ofB << cosB << std::endl;
      if (cosA < 0.4 && cosB < 0.4)
        leaps_[i] = 1;
    }
  }
  ofA.close();
  ofB.close();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  leap_indices_.clear();
  for (int i = 0; i < grid_size_; i++)
  {
    if (leaps_[i] == 1)
      leap_indices_.push_back(i);
  }
  PCL_INFO ("%d leaps detected!\n", leap_indices_.size());
  cloud->resize(leap_indices_.size());
  cloud->points.clear();
  for (std::vector<int>::iterator it = leap_indices_.begin(); it != leap_indices_.end(); it++)
  {
    cloud->points.push_back(ognzd_input_->points[*it]);
  }
  cloud->height = 1;
  cloud->width = leap_indices_.size();
  pcl::visualization::PCLVisualizer viewer ("cloud and detected leaps");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addPointCloud (ognzd_input_, "cloud", 0);
  viewer.addPointCloud (cloud, "leaps", 0);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "leaps");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "leaps");
  viewer.spin ();
}

void
LeapDetection::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  if (cloud->height == 1)
  {
    PCL_ERROR("Input cloud need to be organized!");
    return;
  }
  else
  {
    ognzd_input_ = cloud;
    grid_size_ = cloud->points.size();
    height_ = cloud->height;
    width_ = cloud->width;
  }
}

void LeapDetection::getUognzdCloud()
{
  valid_ = new int [grid_size_];
  memset (valid_, 0, grid_size_ * sizeof (int));
  uognzd_input_->points.clear();
  uognzd_input_->points.resize(grid_size_);
  ognzd_indices_in_uognzd_cloud_.clear();
  ognzd_indices_in_uognzd_cloud_.resize(grid_size_, 0);
  uognzd_indices_in_ognzd_cloud_.clear();
  uognzd_indices_in_ognzd_cloud_.resize(grid_size_, 0);

  int cnt = 0;
  for (int i = 0; i < grid_size_; i++)
  {
    if (ognzd_input_->points[i].x != 0.0 ||
        ognzd_input_->points[i].y != 0.0 ||
        ognzd_input_->points[i].z != 0.0)
    {
      uognzd_input_->points[cnt] = ognzd_input_->points[i];
      ognzd_indices_in_uognzd_cloud_[cnt] = i;
      uognzd_indices_in_ognzd_cloud_[i] = cnt;
      valid_[i] = true;
      cnt ++;
    }
  }

  uognzd_input_->points.erase(uognzd_input_->points.begin() + cnt, uognzd_input_->points.end());
  uognzd_input_->height = 1;
  uognzd_input_->width = cnt;
  ognzd_indices_in_uognzd_cloud_.erase(ognzd_indices_in_uognzd_cloud_.begin() + cnt, ognzd_indices_in_uognzd_cloud_.end());
}
