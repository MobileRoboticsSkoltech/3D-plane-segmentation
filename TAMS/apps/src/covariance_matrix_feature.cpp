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
#include <vector>
#include <cmath>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
//Eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>


void rotationMatrixFromEulerAngles(Eigen::Matrix3d &rm, const double alpha, const double beta, const double gamma);

int
main (int argc, char **argv)
{
  using namespace Eigen;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string pcd_file(argv[1]);
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
            cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());

  Vector3d sum = Vector3d::Zero();
  Matrix3d scatter_matrix = Matrix3d::Zero();
  Vector3d mass_center = Vector3d::Zero();
  //Vector3d mass_center = Vector3d::Zero();
  EigenSolver<Matrix3d> eigensolver;
  Vector3d eigenvalues = Vector3d::Zero();
  Matrix3d eigenvectors = Matrix3d::Zero();
  std::vector<Vector3d, aligned_allocator<Vector3d> > points;
  points.resize(cloud->size());

  for (size_t i = 0; i < cloud->size(); i++)
  {
    points[i](0) = cloud->points[i].x;
    points[i](1) = cloud->points[i].y;
    points[i](2) = cloud->points[i].z;
  }

  for (size_t i = 0; i < points.size(); i++)
  {
    sum += points[i];
  }
  mass_center = sum / points.size();

  for (size_t i = 0; i < points.size(); i++)
  {
    scatter_matrix += (points[i] - mass_center) * (points[i] - mass_center).transpose ();
  }

  scatter_matrix = scatter_matrix / points.size();
  std::cout << "----------------before rotation--------------------\n";
  std::cout << "scatter_matrixariance matrix:\n" << scatter_matrix << std::endl;
  std::cout << "scatter_matrixariance values:\n" <<
               scatter_matrix(0,1) / sqrt(scatter_matrix(0,0)) / sqrt(scatter_matrix(1,1)) << "  " <<
               scatter_matrix(0,2) / sqrt(scatter_matrix(0,0)) / sqrt(scatter_matrix(2,2)) << "  " <<
               scatter_matrix(1,2) / sqrt(scatter_matrix(1,1)) / sqrt(scatter_matrix(2,2)) << std::endl;

  eigensolver.compute(scatter_matrix);
  eigenvalues = eigensolver.eigenvalues().real();
  eigenvectors = eigensolver.eigenvectors().real();
  std::cout << "eigenvalues:" << eigenvalues(0) << " " << eigenvalues(1) << " " << eigenvalues(2) << std::endl;
  double tmp_scaler;
  Vector3d tmp_vector;
  for (int i = 0; i < 3; i++)
  {
    for (int j = i; j < 3; j++)
    {
      if (eigenvalues(i) < eigenvalues(j))
      {
        tmp_scaler = eigenvalues(i);
        tmp_vector = eigenvectors.col(i);
        eigenvalues(i) = eigenvalues(j);
        eigenvectors.col(i) = eigenvectors.col(j);
        eigenvalues(j) = tmp_scaler;
        eigenvectors.col(j) = tmp_vector;
      }
    }
  }

  Matrix3d rotation_matrix = eigenvectors.transpose();
  Vector3d translation = -rotation_matrix * mass_center;
  std::vector<Vector3d, aligned_allocator<Vector3d> > rotated;
  rotated.resize(points.size());
  for (size_t i = 0; i < points.size(); i++)
  {
    rotated[i] = rotation_matrix * points[i] + translation;
  }
  std::cout << "desired scatter_matrixariance matrix:\n" << rotation_matrix * scatter_matrix * rotation_matrix.transpose() << std::endl;
  sum = Vector3d::Zero();
  scatter_matrix = Matrix3d::Zero();
  for (size_t i = 0; i < rotated.size(); i++)
  {
    sum += rotated[i];
  }
  mass_center = sum / rotated.size();

  for (size_t i = 0; i < rotated.size(); i++)
  {
    scatter_matrix += (rotated[i] - mass_center) * (rotated[i] - mass_center).transpose ();
  }

  scatter_matrix = scatter_matrix / rotated.size();
  std::cout << "resulted scatter_matrixariance matrix:\n" << scatter_matrix << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  output->resize(points.size());
  output->points.clear();
  output->points.resize(points.size());
  for (size_t i = 0; i < points.size(); i++)
  {
    output->points[i].x = rotated[i](0);
    output->points[i].y = rotated[i](1);
    output->points[i].z = rotated[i](2);
  }
  output->height = 1;
  output->width = points.size();

  int viewports[2] = {1,2};
  pcl::visualization::PCLVisualizer viewer ("pcd and rotated pcd");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, viewports[0]);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, viewports[1]);
  viewer.addPointCloud (cloud, "cloud", viewports[0]);
  viewer.addPointCloud (output, "EGI", viewports[1]);
  viewer.addCoordinateSystem (0.1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "EGI");
  viewer.spin ();
  return 0;
}

void rotationMatrixFromEulerAngles(Eigen::Matrix3d &rm, const double alpha, const double beta, const double gamma)
{
  double calpha = cos(alpha);
  double salpha = sin(alpha);
  double cbeta = cos(beta);
  double sbeta = sin(beta);
  double cgamma = cos(gamma);
  double sgamma = sin(gamma);
  rm(0,0) = calpha * cbeta * cgamma - salpha * sgamma;
  rm(0,1) = -calpha * cbeta * sgamma - salpha * cgamma;
  rm(0,2) = calpha * sbeta;
  rm(1,0) = salpha * cbeta * cgamma + calpha * sgamma;
  rm(1,1) = -salpha * cbeta * sgamma + calpha * cgamma;
  rm(1,2) = salpha * sbeta;
  rm(2,0) = -sbeta * cgamma;
  rm(2,1) = sbeta * sgamma;
  rm(2,2) = cbeta;
}

