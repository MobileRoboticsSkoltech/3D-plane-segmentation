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

#ifndef PLANAR_PATCH_H_
#define PLANAR_PATCH_H_
#include <vector>
#include <eigen3/Eigen/Core>
#include <boost/shared_ptr.hpp>

namespace tams
{
  using namespace Eigen;
  struct PlanarSegment
  {
    public:
      typedef boost::shared_ptr<PlanarSegment> Ptr;
      typedef std::vector<PlanarSegment, Eigen::aligned_allocator<PlanarSegment> > StdVector;
      typedef boost::shared_ptr<StdVector> StdVectorPtr;
      PlanarSegment () :
        sum (Vector3d::Zero ()), mass_center (Vector3d::Zero ()), normal (Vector3d::Zero ()), second_moment (Matrix3d::Zero ()),
        scatter_matrix (Matrix3d::Zero ()), Cnn(Matrix3d::Zero()), hessian(Matrix4d::Zero ()), covariance (Matrix4d::Zero()),
        bias (0.0), mse (0.0), area (0.0), Cdd (0.0), Cnn_trace (0.0), Dcovariance (0.0), point_num (0){
      }
      Vector3d sum;
      Vector3d mass_center;
      Vector3d normal;
      Matrix3d second_moment;
      Matrix3d scatter_matrix;
      Matrix3d Cnn;
      Matrix4d hessian;
      Matrix4d covariance;
      double bias;
      double mse;
      double area;
      double Cdd;
      double Cnn_trace;
      double Dcovariance;
      int point_num;
      std::vector<int> points;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
