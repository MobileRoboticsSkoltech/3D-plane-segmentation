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

#ifndef COMMON_H_
#define COMMON_H_
//STL
#include <vector>
//PCL
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
//tams
#include "common/planar_patch.h"
#include "common/rgb.h"
#include "Eigen/Core"

namespace tams
{
  void
  getColors(std::vector<RGB> &colors);

  RGB
  getHeatColour (double gray);

  void
  randomColours (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
                 PlanarSegment::StdVectorPtr planar_patches,
                 double min_area,
                 bool project2plane = true);

	/** Converts a rotation-matrix to roll-pitch-yaw. This is a fixed-axes rotation type.
	 * \return false if a singularity (gimbal-lock) was encountered, i.e., the pitch angle has come within
	 * tolerance (radians) near +-Pi/2.
	 * If the conversion failed, roll and yaw are set to 0.
	 * \param[in] rotaton a given 3X3 rotation matrix
	 * \param[out] roll, pitch, yaw the resulted angles
	 */
	bool
  gerRPYFromRotationMatrix(Eigen::Matrix3d rotation,
													 double &roll,
													 double &pitch,
													 double &yaw,
													 double tolerance = M_PI/18000.0 );

}

#endif

