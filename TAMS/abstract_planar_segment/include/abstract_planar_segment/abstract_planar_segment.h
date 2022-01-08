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

#ifndef ABSTRACT_PLANAR_SEGMENT_H_
#define ABSTRACT_PLANAR_SEGMENT_H_

//STL
#include <vector>
//boost
#include <boost/shared_ptr.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

#include "common/planar_patch.h"

namespace tams
{
  class AbstractPlanarSegment
  {
    public:
      /** Typedefs. */
      typedef boost::shared_ptr<AbstractPlanarSegment> Ptr;
      typedef boost::shared_ptr<const AbstractPlanarSegment> ConstPtr;
      typedef std::vector<AbstractPlanarSegment, Eigen::aligned_allocator<AbstractPlanarSegment> > StdVector;
      typedef boost::shared_ptr<StdVector> StdVectorPtr;

    public:
      /** \brief Empty constructor. */
      AbstractPlanarSegment():
        a0(0.0), a1(0.0), a2(0.0), normal(Eigen::Vector3d::Zero ()), d(0.0), weighted_center(Eigen::Vector3d::Zero ()),
        scatter_matrix(Eigen::Matrix3d::Zero ()), hessian(Eigen::Matrix4d::Zero()), C4x4(Eigen::Matrix4d::Zero ()), Cnn(Eigen::Matrix3d::Zero ()),
        Cdd(0.0), minusLogDeterminantC (0.0), minusLogDeterminantCnn (0.0), area (0.0)
      {

      }

      /** Empty deconstructor. */
      ~AbstractPlanarSegment()
      {

      }

      /** \brief Set the noisy model of the corresponding range sensor.
          sensor noise delta = a0_ + a_1_ * range + a_2_ * range * range.
        */
      void
      setSensorNoiseModel(double b0, double b1, double b2)
      {
        a0= b0;
        a1 = b1;
        a2 = b2;
      }

      /** \brief Compute the attributes for the given segement w.r.t the corresponding point cloud. */
      void
      calculateAttributes(PlanarSegment::StdVector::iterator segment, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    public:
      /** \brief Hessian form plane equation: n.dot(p) = d. */
      Eigen::Vector3d normal;
      /** \brief Plane equation: n.dot(p) = d. */
      double d;
      /** \brief Weighted center of the segment. */
      Eigen::Vector3d weighted_center;
      /** \brief Scatter matrix of 3D points of the segment. */
      Eigen::Matrix3d scatter_matrix;
      /** \brief Hessian matrix of least square fitting. */
      Eigen::Matrix4d hessian;
      /** \brief Plane parameter covariance matrix.*/
      Eigen::Matrix4d C4x4;
      /** \brief Normal vector estimation covariance matrix.*/
      Eigen::Matrix3d Cnn;
      /** \brief Estimation variance of the distance from origin, see plane equation. */
      double Cdd;
      /** \brief minusLogDeterminantC_ = -(log(eigenvalue(C4x4_,0)) + log(eigenvalue(C4x4_,1)) + log(eigenvalue(C4x4_,2))),
          with the condition eigenvalue(C4x4_,3) = 0.*/
      double minusLogDeterminantC;
      /** \brief minusLogDeterminantCnn_ = -(log(eigenvalue(Cnn_,0)) + log(eigenvalue(Cnn_,1))),
          with the condition eigenvalue(C4x4_,2) = 0.*/
      double minusLogDeterminantCnn;
      /** \brief Area covered by the points on the segment. */
      double area;
      /** \brief Number of points in this segment. */
      size_t point_num;
      /** \brief sensor noise delta = a0_ + a_1_ * range + a_2_ * range * range. */
      double a0, a1, a2;
      /** \brief a metric to measure how linear (long-thin) the segment is. */
      double linearity;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // ABSTRACT_PLANAR_SEGMENT_H_
