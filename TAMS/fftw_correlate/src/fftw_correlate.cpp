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

#include "fftw_correlate/fftw_correlate.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <algorithm>

void
FFTWCorrelate::softFFTWCorrelateReal()
{
  float tstart = csecond ();
  FILE *fp;
  softFFTWCorrelate(bwIn_, egi_map_, egi_data_, &alpha_, &beta_, &gamma_, 1) ;
  printf("alpha = %f\nbeta = %f\ngamma = %f\n", alpha_, beta_, gamma_);
  float tstop = csecond ();
  PCL_INFO ("%f seconds elapsed.\n", tstop - tstart);
  return;
}

void
FFTWCorrelate::rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                 float *rm)
{
  output->resize(input->size());
  for (size_t i = 0; i < input->size(); i++)
  {
    output->points[i].x = rm[0] * input->points[i].x + rm[1] * input->points[i].y + rm[2] * input->points[i].z;
    output->points[i].y = rm[3] * input->points[i].x + rm[4] * input->points[i].y + rm[5] * input->points[i].z;
    output->points[i].z = rm[6] * input->points[i].x + rm[7] * input->points[i].y + rm[8] * input->points[i].z;
  }
}

void
FFTWCorrelate::heatmapRGB (float gray, uint8_t &r, uint8_t &g, uint8_t &b)
{
  if (gray >= 0.0 && gray <= 0.125)
  {
    r = 0;
    g = 0;
    b = 127 + floor (gray * 128 / 0.125);
  }
  else if (gray > 0.125 && gray <= 0.375)
  {
    r = 0;
    g = floor ((gray - 0.125) * 255 / 0.25);
    b = 255;
  }
  else if (gray > 0.375 && gray <= 0.625)
  {
    r = floor ((gray - 0.375) * 255 / 0.25);
    g = 255;
    b = 255 - floor ((gray - 0.375) * 255 / 0.25);
  }
  else if (gray > 0.625 && gray <= 0.875)
  {
    r = 255;
    g = 255 - floor ((gray - 0.625) * 255 / 0.25);
    b = 0;
  }
  else if (gray > 0.875 && gray <= 1.0)
  {
    r = 255 - floor ((gray - 0.875) * 128 / 0.125);
    g = 0;
    b = 0;
  }
}

void
FFTWCorrelate::egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             float *egi,
                             const int bandwidth,
                             const bool organized,
                             const bool write2file,
                             const bool visualization)
{
  memset (egi, 0.0, sizeof (float) * 4 * bwIn_ * bwIn_);
  if (organized == true)
  {
    egiFromNormal(cloud, egi, bandwidth, visualization);
    return;
  }
  boost::timer t;
  t.restart();

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  // Use all neighbors in a sphere of radius 3cm
  //ne.setRadiusSearch (0.3);
  ne.setKSearch (50);
  // Compute the features#
  ne.compute (*cloud_normals);
  // Construct the egis for given point cloud
  // make sample grid for bandwidth 128

  float egi_resolution_theta = M_PI / bandwidth / 2;
  float egi_resolution_phi = M_PI / bandwidth;
  float pcd_resolution_theta = 0.125;
  float tan_theta_step = tan (pcd_resolution_theta * M_PI / 180);
  float pcd_resolution_phi = 0.25;
  float tan_phi_step = tan (pcd_resolution_phi * M_PI / 180);
  float *pcd_theta = new float[cloud->size ()];
  float *pcd_phi = new float[cloud->size ()];
  for (size_t i = 0; i < cloud->size (); i++)
  {
    pcd_theta[i] = acos (-cloud_normals->points[i].normal_z);
    pcd_phi[i] = atan2 (-cloud_normals->points[i].normal_y, -cloud_normals->points[i].normal_x);
    if (pcd_phi[i] < 0.0)
      pcd_phi[i] = pcd_phi[i] + 2 * M_PI;
  }
  float area = 0.0;
  int u = 0, v = 0;
  for (size_t i = 0; i < cloud_normals->size (); i++)
  {
    area = 1;
    u = floor (pcd_theta[i] / egi_resolution_theta);
    v = floor (pcd_phi[i] / egi_resolution_phi);
    egi[u * 2 * bandwidth + v] += area;
  }

  delete [] pcd_theta;
  delete [] pcd_phi;
  PCL_INFO ("%f seconds elapsed for constructing EGI for the point cloud.\n", t.elapsed());
  if (write2file)
  {
    std::ofstream egi_file;
    egi_file.open ("scan001.egi");
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      egi_file << egi[i] << std::endl;
    }
    egi_file.close ();
  }
  if (visualization)
  {
    float max = 0.0;
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      if (egi[i] > max)
      {
        max = egi[i];
      }
    }
    float *gray = new float[4 * bandwidth * bandwidth];
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      gray[i] = egi[i] / max;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere->resize (4 * bandwidth * bandwidth);
    uint8_t r = 0, g = 0, b = 0;
    uint32_t rgb = 0;
    float tmp_theta = 0.0;
    float tmp_phi = 0.0;
    for (size_t i = 0; i < 2 * bandwidth; i++)
    {
      tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
      for (size_t j = 0; j < 2 * bandwidth; j++)
      {
        tmp_phi = M_PI * j / bandwidth;
        sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
        heatmapRGB (gray[i * 2 * bandwidth + j], r, g, b);
        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
      }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals (new pcl::PointCloud<pcl::PointXYZ>);
    normals->resize (cloud_normals->size ());
    for (size_t i = 0; i < cloud_normals->size (); i++)
    {
      normals->points[i].x = -cloud_normals->points[i].normal_x;
      normals->points[i].y = -cloud_normals->points[i].normal_y;
      normals->points[i].z = -cloud_normals->points[i].normal_z;
    }

    delete [] gray;

    int viewports[2] = {1, 2};
    pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, viewports[0]);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, viewports[1]);
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloud (normals, "normals", viewports[0]);
    viewer.addPointCloud (sphere, "EGI", viewports[1]);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normals");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "EGI");
    viewer.spin ();
  }
  return;
}

void
FFTWCorrelate::constellationImage(const std::string segments_file,
                                  const int bandwidth,
                                  float *egi)
{
  memset (egi, 0.0, sizeof (float) * 4 * bwIn_ * bwIn_);
  float egi_resolution_theta = M_PI / bandwidth / 2;
  float egi_resolution_phi = M_PI / bandwidth;
  float tmp;
  float area;
  int point_num;
  float theta, phi;
  std::ifstream file_in;
  file_in.open(segments_file.c_str());
  file_in >> tmp >> tmp >> tmp >> point_num;
  Eigen::Vector3f normal(Eigen::Vector3f::Zero());
  for (size_t i = 0; i < point_num; i++)
  {
    file_in >> normal(0) >> normal(1) >> normal(2) >> tmp >> tmp >> area >> point_num >> tmp >> tmp >> tmp;
    if (area > 0.0)
    {
      theta = acos (normal(2));
      phi = atan2 (normal(1), normal(0));
      if (phi < 0.0)
        phi = phi + 2 * M_PI;
      int u = floor (theta / egi_resolution_theta);
      int v = floor (phi / egi_resolution_phi);
      egi[u * 2 * bandwidth + v] += area;
    }
  }
  file_in.close ();
}


void
FFTWCorrelate::egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             float *egi,
                             const int bandwidth,
                             const bool visualization)
{
  using namespace Eigen;
  memset (egi, 0.0, sizeof (float) * bwIn_ * bwIn_);

  int height = cloud->height;
  int width = cloud->width;

  bool *valid = new bool [height * width];
  memset (valid, false, height * width * sizeof (bool));
  std::vector<Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > points;
  points.resize(height * width, Vector3f::Zero());
  std::vector<Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > normals;
  normals.resize(height * width, Vector3f::Zero());
  bool *has_normal = new bool [height * width];
  memset (has_normal, false, height * width * sizeof (bool));
  float *rho = new float [height * width];
  memset (rho, 0.0, height * width * sizeof (float));
  for (size_t i = 0; i < cloud->height * cloud->width; i++)
  {
    if (cloud->points[i].z < 0 &&
        (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y) < 0.64)
    {
      cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = 0;
    }
    else if (cloud->points[i].x * cloud->points[i].x +
             cloud->points[i].y * cloud->points[i].y +
             cloud->points[i].z * cloud->points[i].z > 29 * 29)
    {
      cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = 0;
    }
  }

  for (size_t i = 0; i < cloud->height * cloud->width; i++)
  {
    if (cloud->points[i].x != 0.0 || cloud->points[i].y != 0.0 || cloud->points[i].z != 0.0)
    {
      points[i](0) = cloud->points[i].x;
      points[i](1) = cloud->points[i].y;
      points[i](2) = cloud->points[i].z;
      rho[i] = points[i].norm();
      valid[i] = true;
    }
  }

//  float *radius_dis = new float [height * width];
//  memset (radius_dis, 0.0, height * width * sizeof(float));
//  std::vector<float> dis(8, 0.0);
//  for (int i = 1; i < height - 1; i++)
//  {
//    for (int j = 1; j < width - 1; j++)
//    {
//      int index = i * width + j;
//      if (valid[index])
//      {
//        dis[0] = valid[index - width - 1] ? fabs(rho[index] - rho[index - width - 1]) : 0.0;
//        dis[1] = valid[index - width] ? fabs(rho[index] - rho[index - width]) : 0.0;
//        dis[2] = valid[index - width + 1] ? fabs(rho[index] - rho[index - width + 1]) : 0.0;
//        dis[3] = valid[index -1] ? fabs(rho[index] - rho[index -1]) : 0.0;
//        dis[4] = valid[index + 1] ? fabs(rho[index] - rho[index + 1]): 0.0;
//        dis[5] = valid[index + width - 1] ? fabs(rho[index] - rho[index + width - 1]) : 0.0;
//        dis[6] = valid[index + width] ? fabs(rho[index] - rho[index + width]) : 0.0;
//        dis[7] = valid[index + width + 1] ? fabs(rho[index] - rho[index + width + 1]) : 0.0;
//        radius_dis[index] = *std::min_element(dis.begin(), dis.end());
//      }
//    }
//  }

  int subwindow_length = 7;
  Vector3f sum;
  Vector3f normal;
  Matrix3f scatter_matrix;
  Vector3f mass_center;
  EigenSolver<Matrix3f> eigensolver;
  Vector3f eigenvalues = Vector3f::Zero();
  Matrix3f eigenvectors = Matrix3f::Zero();

  int valid_cnt = 0;
  int row = 0, col = 0;
  int grid_size = (2 * subwindow_length + 1) * (2 * subwindow_length + 1);
  int *indices = new int [grid_size];
  for (int i = subwindow_length; i < width - subwindow_length; i++)
  {
    for (int j = subwindow_length; j < height - subwindow_length; j++)
    {
       if (!valid[j * width + i])
         continue;
       memset(indices, 0, grid_size * sizeof (int));
       sum = Vector3f::Zero();
       scatter_matrix = Matrix3f::Zero();
       valid_cnt = 0;
       int col_start = i - subwindow_length;
       int col_end = i + subwindow_length;
       int row_start = j - subwindow_length;
       int row_end   = j + subwindow_length;
       int t = 0;
       for (col = col_start; col <= col_end; col++)
       {
         for (row = row_start; row <= row_end; row++)
         {
           indices[t++] = row * width + col;
         }
       }
       for (int k = 0; k < grid_size; k++)
       {
         if (valid[indices[k]])
         {
           valid_cnt++;
         }
       }
       //if (valid_cnt > grid_size * 0.7)
       {
         for (int k = 0; k < grid_size; k++)
         {
           if (valid[indices[k]])
             sum += points[indices[k]];
         }
         mass_center = sum / valid_cnt;
         for (int k = 0; k < grid_size; k++)
         {
           if (valid[indices[k]] )
             scatter_matrix += (points[indices[k]] - mass_center) * (points[indices[k]] - mass_center).transpose();
         }
         eigensolver.compute(scatter_matrix);
         eigenvalues = eigensolver.eigenvalues().real();
         eigenvectors = eigensolver.eigenvectors().real();
         int min_eigenvalue_index;
         float lambda[2];
         float min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
         normal = eigenvectors.col(min_eigenvalue_index);
         if (normal.dot(mass_center) < 0)
           normal = -normal;
         has_normal[j * width + i] = true;
         normals[j * width + i] = normal;
       }
    }
  }
  float *pcd_theta = new float [height * width];
  float *pcd_phi = new float [height * width];
  float *area = new float [height * width];
  memset (area, 0.0, height * width * sizeof (float));

  for (size_t i = 0; i < height * width; i++)
  {
    if (has_normal[i] && valid[i + 1] && valid[i + width] && valid[i + width +1])
    {
      area[i] += fabs(normals[i].dot(
          points[i].cross(points[i + width]) +
          points[i + width].cross(points[i + width + 1]) +
          points[i + width + 1].cross(points[i + 1]) +
          points[i + 1].cross(points[i])));
    }
  }

  for (size_t i = 0; i < height * width; i++)
  {
    if (has_normal[i])
    {
      pcd_theta[i] = acos (normals[i](2));
      pcd_phi[i] = atan2 (normals[i](1), normals[i](0));
      if (pcd_phi[i] < 0.0)
        pcd_phi[i] = pcd_phi[i] + 2 * M_PI;
    }
  }

  float egi_resolution_theta = M_PI / bandwidth / 2;
  float egi_resolution_phi = M_PI / bandwidth;

  int u = 0, v = 0;
  for (size_t i = 0; i < height * width; i++)
  {
    if (has_normal[i])
    {
      u = floor (pcd_theta[i] / egi_resolution_theta);
      v = floor (pcd_phi[i] / egi_resolution_phi);
      egi[u * 2 * bandwidth + v] += area[i];
    }
  }

  delete [] indices;
  delete [] valid;
  delete [] has_normal;
  delete [] pcd_theta;
  delete [] pcd_phi;
  delete [] area;

  if (visualization)
  {
    float max = 0.0;
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      if (egi[i] > max)
      {
        max = egi[i];
      }
    }
    float *gray = new float[4 * bandwidth * bandwidth];
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      gray[i] = egi[i] / max;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere->resize (4 * bandwidth * bandwidth);
    uint8_t r = 0, g = 0, b = 0;
    uint32_t rgb = 0;
    float tmp_theta = 0.0;
    float tmp_phi = 0.0;
    for (size_t i = 0; i < 2 * bandwidth; i++)
    {
      tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
      for (size_t j = 0; j < 2 * bandwidth; j++)
      {
        tmp_phi = M_PI * j / bandwidth;
        sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
        heatmapRGB (gray[i * 2 * bandwidth + j], r, g, b);
        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
      }
    }
    delete [] gray;
    pcl::visualization::PCLVisualizer viewer ("EGI");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloud (sphere, "EGI");
    //viewer.addCoordinateSystem (0.5);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "EGI");
    viewer.spin ();
  }

  return;
}

void
FFTWCorrelate::egiMap (bool organized, bool write2file, bool visualization)
{
  egiFromNormal(cloud_map_, egi_map_, bwIn_, organized, write2file, visualization);
}

void
FFTWCorrelate::egiData (bool organized, bool write2file, bool visualization)
{
  egiFromNormal(cloud_data_, egi_data_, bwIn_, organized, write2file, visualization);
}

void
FFTWCorrelate::visualize()
{
  rotationMatrixFromEulerAngles(rm_, alpha_, beta_, gamma_);
  rotatePointcloud(cloud_data_, rotated_cloud_data_, rm_);

  pcl::visualization::PCLVisualizer viewer ("cloud and rotated cloud");
  int viewports[2] = {1,2};
  viewer.createViewPort(0, 0, 0.5, 1.0, viewports[0]);
  viewer.createViewPort(0.5, 0, 1.0, 1.0, viewports[1]);

  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  //viewer.addCoordinateSystem (1.0);
  viewer.setCameraPosition (30.0, 30.0, 10.0, 0.0, 0.0, 0.0);

  viewer.addPointCloud(cloud_map_, "map", viewports[0]);
  viewer.addPointCloud (cloud_data_, "data", viewports[0]);

  viewer.addPointCloud(cloud_map_, "rotated_map", viewports[1]);
  viewer.addPointCloud(rotated_cloud_data_, "rotated_data", viewports[1]);

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "data");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "map");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "data");

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_map");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_data");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "rotated_map");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "rotated_data");

  viewer.spin();

}

void
FFTWCorrelate::initialize ()
{
  egi_map_ = new float [bwIn_ * bwIn_ * 4];
  egi_data_ = new float [bwIn_ * bwIn_ * 4];
  /*rms_ = new float [359 * 89 * 359 * 9];
  rms_ = (float *) malloc (sizeof(float) * 359 * 89 * 359 * 9);
  if (rms_ == NULL)
    std::cout << "bad malloc!\n";
  float calpha;
  float salpha;
  float cbeta;
  float sbeta;
  float cgamma;
  float sgamma;
  float alpha;
  float beta;
  float gamma;
  float radian_per_degree = M_PI / 180;
  int index;
  for (int i = 1; i < 360; i++)
  {
    for (int j = 1; j < 90; j++)
    {
      for (int k = 1; k < 360; k++)
      {
        alpha = i * radian_per_degree;
        beta = j * radian_per_degree;
        gamma = k * radian_per_degree;
        float calpha = cos(alpha);
        float salpha = sin(alpha);
        float cbeta = cos(beta);
        float sbeta = sin(beta);
        float cgamma = cos(gamma);
        float sgamma = sin(gamma);
        index = i * 287559 + j * 3231 + k * 9;
        rms_[index    ] =  calpha * cbeta * cgamma - salpha * sgamma;
        rms_[index + 1] = -calpha * cbeta * sgamma - salpha * cgamma;
        rms_[index + 2] =  calpha * sbeta;
        rms_[index + 3] =  salpha * cbeta * cgamma + calpha * sgamma;
        rms_[index + 4] = -salpha * cbeta * sgamma + calpha * cgamma;
        rms_[index + 5] =  salpha * sbeta;
        rms_[index + 6] = -sbeta  * cgamma;
        rms_[index + 7] =  sbeta  * sgamma;
        rms_[index + 8] =  cbeta;
      }
    }
  }*/
}

void FFTWCorrelate::test()
{
  struct timeval tpstart,tpend;
  float timeuse;
  gettimeofday(&tpstart,NULL);
  float in[3] = {0, 0.707, 0.707};
  float output[3];
  for (int j = 0; j < 1; j++)
  {
    for (int i = 0; i < 359 * 89 * 359 * 9; i+=9)
    {
      rms_[i + 6] = i + j;
      rms_[i + 7] = i + 2 * j;
      rms_[i + 8] = i + 3 * j;
    }
  }
  gettimeofday(&tpend,NULL);
  timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
  timeuse/=1000000;
  printf("Used Time:%f\n",timeuse);
  for (int i = 359 * 89 * 359 * 9 - 81; i < 359 * 89 * 359 * 9; i += 9)
  {
    std::cout << rms_[i + 6] << "  " << rms_[i + 7] << " " << rms_[i + 8] << std::endl;
  }
}

void
FFTWCorrelate::rotationMatrixFromEulerAngles(float *rm, const float alpha, const float beta, const float gamma)
{
  float calpha = cos(alpha);
  float salpha = sin(alpha);
  float cbeta = cos(beta);
  float sbeta = sin(beta);
  float cgamma = cos(gamma);
  float sgamma = sin(gamma);
  rm[0] = calpha * cbeta * cgamma - salpha * sgamma;
  rm[1] = -calpha * cbeta * sgamma - salpha * cgamma;
  rm[2] = calpha * sbeta;
  rm[3] = salpha * cbeta * cgamma + calpha * sgamma;
  rm[4] = -salpha * cbeta * sgamma + calpha * cgamma;
  rm[5] = salpha * sbeta;
  rm[6] = -sbeta * cgamma;
  rm[7] = sbeta * sgamma;
  rm[8] = cbeta;
}

void
FFTWCorrelate::dataAsMap ()
{
  *cloud_map_ = *cloud_data_;
  memcpy (egi_map_, egi_data_, bwIn_ * bwIn_ * 4 * sizeof (float));
}
