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

#ifndef SUB_WINDOW_H_H
#define SUB_WINDOW_H_H
#include <vector>
#include <Eigen/Core>
namespace tams
{
using namespace Eigen;
struct Subwindow
{
  typedef boost::shared_ptr<Subwindow> Ptr;
  typedef std::vector<Subwindow> StdVector;
  typedef boost::shared_ptr<StdVector> StdVectorPtr;
  Vector3d sum;
  Vector3d mass_center;
  Vector3d normal;
  Matrix3d second_moment;
  int point_num;
  double bias;
  double mse;
  std::vector<int> points;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Subwindow ():
  sum (Vector3d::Zero()), mass_center (Vector3d::Zero()), 
  normal (Vector3d::Zero()), second_moment (Matrix3d::Zero()),
  point_num (0), bias (0.0), mse (0.0) {}
  ~Subwindow () {}
};
}
#endif
