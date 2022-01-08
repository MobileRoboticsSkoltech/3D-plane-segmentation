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

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <abstract_planar_segment/abstract_planar_segment.h>

namespace tams
{
  void
  AbstractPlanarSegment::calculateAttributes(PlanarSegment::StdVector::iterator segment, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    if (a0 == 0.0 && a1 == 0.0 && a2 == 0.0)
    {
      std::cerr << "You have not set the sensor noise parameters a0_, a1_ and a2_, set them using setSensorNoiseModel(a0_, a1_, a2_)." << std::endl;
      return;
    }

    double weight_sum;
    Eigen::Vector3d weighted_sum;
    /** the weight for each point. */
    std::vector<double > weights;
    /** the scatter matrix. */
    Eigen::Matrix3d S, Hnn;
    Eigen::Vector3d Hnd;
    double Hdd;

    Eigen::EigenSolver<Eigen::Matrix4d> eigensolver4d;
    Eigen::Vector4d eigenvalues4f = Eigen::Vector4d::Zero();
    Eigen::Matrix4d eigenvectors4d = Eigen::Matrix4d::Zero();
    Eigen::EigenSolver<Eigen::Matrix3d> eigensolver3d;
    Eigen::Vector3d eigenvalues3d = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors3d = Eigen::Matrix3d::Zero();
    int min_eigenvalue_index, max_eigenvalue_index;
    double min_eigenvalue, max_eigenvalue;
    PlanarSegment::StdVector::iterator it;

    weights.clear();
    weights.resize(segment->points.size());
    weight_sum = 0.0;
    weighted_sum = Eigen::Vector3d::Zero();
    S = Eigen::Matrix3d::Zero ();
    double squared_norm = 0.0;
    size_t i = 0;
    for (std::vector<int>::iterator it = segment->points.begin(); it != segment->points.end(); it++, i++)
    {
      squared_norm = cloud->points[*it].x * cloud->points[*it].x +
                     cloud->points[*it].y * cloud->points[*it].y +
                     cloud->points[*it].z * cloud->points[*it].z;
      weights[i] = a0 + a1 * sqrt(squared_norm) + a2 * squared_norm;
      weights[i] = 1 / (weights[i] * weights[i]);
      weight_sum += weights[i];
      weighted_sum += weights[i] * Eigen::Vector3d(cloud->points[*it].x, cloud->points[*it].y, cloud->points[*it].z);
    }
    weighted_center = weighted_sum / weight_sum;
    i = 0;
    for (std::vector<int>::iterator it = segment->points.begin(); it != segment->points.end(); it++, i++)
    {
      Eigen::Vector3d tmp(cloud->points[*it].x - weighted_center(0), cloud->points[*it].y - weighted_center(1), cloud->points[*it].z - weighted_center(2));
      S += weights[i] * (tmp * tmp.transpose ());
    }
    eigensolver3d.compute(S);
    eigenvalues3d = eigensolver3d.eigenvalues().real();
    eigenvectors3d = eigensolver3d.eigenvectors().real();
    min_eigenvalue = eigenvalues3d.minCoeff(&min_eigenvalue_index);

    double lambada[2];
    i = 0;
    for (int j = 0; j < 3; j++)
    {
      if (j == min_eigenvalue_index)
        continue;
      lambada[i] = eigenvalues3d[j];
      i ++;
    }

    linearity = lambada[0] / lambada[1];
    if (linearity < 1.0)
      linearity = 1/linearity;

    Hdd = -weight_sum;
    Hnn = -S - weight_sum * weighted_center * weighted_center.transpose() + min_eigenvalue * Eigen::Matrix3d::Identity();
    Hnd = weight_sum * weighted_center;

    hessian.block<3,3>(0,0) = Hnn;
    hessian.block<3,1>(0,3) = Hnd;
    hessian.block<1,3>(3,0) = Hnd.transpose();
    hessian(3,3) = Hdd;

    eigensolver4d.compute(hessian);
    eigenvalues4f = eigensolver4d.eigenvalues().real();
    eigenvectors4d = eigensolver4d.eigenvectors().real();
    max_eigenvalue = eigenvalues4f.cwiseAbs().maxCoeff(&max_eigenvalue_index);
    min_eigenvalue = eigenvalues4f.cwiseAbs().minCoeff(&min_eigenvalue_index);

    //std::cout << "eigenvalues: " << eigenvalues4f.transpose () << std::endl;
    C4x4 = Eigen::Matrix4d::Zero ();
    double DC4x4 = 1.0;
    for (int j = 0; j < 4; j++)
    {
      if (j == min_eigenvalue_index)
        continue;
      C4x4 += eigenvectors4d.col(j) * eigenvectors4d.col(j).transpose () / eigenvalues4f(j);
      DC4x4 /= eigenvalues4f(j);
    }
    C4x4 = -C4x4;
    DC4x4 = -DC4x4;
    //std::cout << DC4x4 << std::endl;

    double factor = 1.0 / sqrt(1 - eigenvectors4d(3, min_eigenvalue_index) * eigenvectors4d(3, min_eigenvalue_index));
    if (eigenvectors4d(3, min_eigenvalue_index) < 0.0)
      factor = -factor;

    normal(0) = factor * eigenvectors4d(0, min_eigenvalue_index);
    normal(1) = factor * eigenvectors4d(1, min_eigenvalue_index);
    normal(2) = factor * eigenvectors4d(2, min_eigenvalue_index);
    double bias = factor * eigenvectors4d(3, min_eigenvalue_index);
    //std::cout << it->normal.dot(normal) << " " << fabs(bias - it->bias) << std::endl;
    Eigen::Matrix3d Hnn_inv = Hnn.inverse ();
    Eigen::Matrix3d Hnn_prime = Hnn - (1/Hdd) * Hnd * Hnd.transpose();

    eigensolver3d.compute(-Hnn_prime);
    eigenvalues3d = eigensolver3d.eigenvalues().real();
    eigenvectors3d = eigensolver3d.eigenvectors().real();
    min_eigenvalue = eigenvalues3d.cwiseAbs().minCoeff(&min_eigenvalue_index);
    Cnn = Eigen::Matrix3d::Zero ();
    for (int j = 0; j < 3; j++)
    {
      if (j == min_eigenvalue_index)
        continue;
      Cnn += eigenvectors3d.col(j) * eigenvectors3d.col(j).transpose() / eigenvalues3d(j);
    }

    Cdd = -normal.dot(Hnn_inv * normal) / (normal.dot(Hnn_inv * Hnd) * normal.dot(Hnn_inv * Hnd));
    /* write the planar patch corresponding attributes. */
    d = bias;

    scatter_matrix = S;
    area = segment->area;
    point_num = segment->point_num;
  }
}
