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

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/surface/mls.h>
#include <boost/timer.hpp>
#include <string>
#include <cmath>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>

extern "C"
{
#include "csecond.h"
#include "makeweights.h"
#include "so3_correlate_fftw.h"
#include "soft_fftw.h"
#include "s2_cospmls.h"
#include "s2_legendreTransforms.h"
#include "s2_semi_memo.h"
#include "wrap_fftw.h"
}

#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )

void
rotationMatrixFromEulerAngles(double*rm, const doublealpha, const doublebeta, const doublegamma);
void
rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                 double*rm);

void softFFTWCorrelateReal(double*signal, double*pattern, const int bw, double&alpha, double&beta, double&gamma);

void softFFTWCorrelate(double*sigR, double*sigI,
                       double*patternR, double*patternI,
                       const int bwIn, const int bwOut, const int degLim, bool wirte2file,
                       double&alpha, double&beta, double&gamma);
void
heatmapRGB (doublegray, uint8_t &r, uint8_t &g, uint8_t &b);
void
egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double*egi, const int bandwidth, const bool write2file, const bool visualization);
void
egiFromRange (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double*egi, const int bandwidth, const bool write2file, const bool visualization);
int
main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointXYZ> points;
  boost::timer t;

  if (argc != 4)
  {
    std::cout << "usage: egi <dir> <start> <end>. \n";
    return -1;
  }
  std::string dir(argv[1]);
  int start = atoi(argv[2]);
  int end = atoi(argv[3]);
  int BW = 128;
  doublealpha;
  doublebeta;
  doublegamma;
  double*rm = new double[9];
  double*egi_map = new double[4 * BW * BW];
  double*egi_data = new double[4 * BW * BW];
  double*egiI = new double[4 * BW * BW];
  memset (egiI, 0.0, 4 * BW * BW * sizeof (double));

  pcl::visualization::PCLVisualizer viewer ("cloud and rotated cloud");
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  viewer.addCoordinateSystem (1.0);
  viewer.setCameraPosition (30.0, 30.0, 10.0, 0.0, 0.0, 0.0);
  for (int scan_index = start; scan_index < end; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string map_file = dir + "/scan" + std::string (buf) + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_file, *map_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", map_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              map_cloud->points.size(), map_cloud->width, map_cloud->height, map_file.c_str());

    sprintf (buf, "%03d", scan_index + 1);
    std::string data_file = dir + std::string("/scan") + std::string(buf) + std::string(".pcd");
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (data_file, *data_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", data_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              data_cloud->points.size(), data_cloud->width, data_cloud->height, data_file.c_str());

    PCL_INFO ("data processing: long range outlier remove start.\n");
    size_t valid_point = 0;
    doublethreshold = 29 * 29;
    points.resize(map_cloud->size());
    for (size_t i = 0; i < map_cloud->size(); i++)
    {
      if (map_cloud->points[i].z < 0 &&
          (map_cloud->points[i].x * map_cloud->points[i].x + map_cloud->points[i].y * map_cloud->points[i].y) < 0.64)
        continue;
      if (map_cloud->points[i].x * map_cloud->points[i].x +
          map_cloud->points[i].y * map_cloud->points[i].y +
          map_cloud->points[i].z * map_cloud->points[i].z > threshold)
        continue;
      points[valid_point] = map_cloud->points[i];
      valid_point ++;
    }
    map_cloud->resize(valid_point);
    map_cloud->points.clear ();
    map_cloud->points.insert (map_cloud->points.begin(), points.begin(), points.begin() + valid_point);
    map_cloud->height = 1;
    map_cloud->width = valid_point;
    std::cerr << "Cloud after long range outlier removing: " << std::endl;
    std::cerr << *map_cloud << std::endl;

    valid_point = 0;
    points.resize(data_cloud->size());
    for (size_t i = 0; i < data_cloud->size(); i++)
    {
      if (data_cloud->points[i].z < 0 &&
          (data_cloud->points[i].x * data_cloud->points[i].x + data_cloud->points[i].y * data_cloud->points[i].y) < 0.64)
        continue;
      if (data_cloud->points[i].x * data_cloud->points[i].x +
          data_cloud->points[i].y * data_cloud->points[i].y +
          data_cloud->points[i].z * data_cloud->points[i].z > threshold)
        continue;
      points[valid_point] = data_cloud->points[i];
      valid_point ++;
    }
    data_cloud->resize(valid_point);
    data_cloud->points.clear ();
    data_cloud->points.insert (data_cloud->points.begin(), points.begin(), points.begin() + valid_point);
    data_cloud->height = 1;
    data_cloud->width = valid_point;
    std::cerr << "Cloud after long range outlier removing: " << std::endl;
    std::cerr << *data_cloud << std::endl;

    memset (egi_map, 0.0, 4 * BW * BW * sizeof (double));
    //egiFromRange(map_cloud, egi_map, BW, false, false);
    egiFromNormal(map_cloud, egi_map, BW, false, false);
    memset (egi_data, 0.0, 4 * BW * BW * sizeof (double));
    //egiFromRange(data_cloud, egi_data, BW, false, false);
    egiFromNormal(data_cloud, egi_data, BW, false, false);
    softFFTWCorrelateReal(egi_map, egi_data, BW, alpha, beta, gamma);
    //softFFTWCorrelate(egi_map, egiI, egi_data, egiI, BW, BW, BW - 1, false, alpha, beta, gamma);
    memset(rm, 0.0, 9 * sizeof(double));
    rotationMatrixFromEulerAngles(rm, alpha, beta, gamma);
    rotatePointcloud(data_cloud, rotated_data_cloud, rm);

    //int viewports[2] = {1,2};
    //viewer.createViewPort(0, 0, 0.5, 1.0, viewports[0]);
    //viewer.createViewPort(0.5, 0, 1.0, 1.0, viewports[1]);
    //viewer.addPointCloud(map_cloud, "cloud", viewports[0]);
    //viewer.addPointCloud (rotated_data_cloud, "rotated_cloud", viewports[1]);
    viewer.addPointCloud(map_cloud, "cloud", 0);
    viewer.addPointCloud(rotated_data_cloud, "rotated_cloud", 0);

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "rotated_cloud");

    //viewer.setFullScreen(true);
    //sprintf(buf, "%03d", scan_index);
    //std::string screenshot = std::string(buf) + "to";
    //sprintf(buf, "%03d", scan_index + 1);
    //screenshot += std::string(buf) + ".png";
    //viewer.saveScreenshot(screenshot);
    viewer.spin();
    //viewer.spinOnce(1);
    viewer.removeAllPointClouds();
    //viewer.spinOnce(1);
  }
  delete [] rm;
  delete [] egi_map;
  delete [] egi_data;
  delete [] egiI;

  return 0;
}

void
egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double*egi, const int bandwidth, const bool write2file, const bool visualization)
{
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
  ne.setRadiusSearch (0.3);
  //ne.setKSearch (50);
  // Compute the features#
  ne.compute (*cloud_normals);
  // Construct the egis for given point cloud
  // make sample grid for bandwidth 128
  doubleegi_resolution_theta = M_PI / bandwidth / 2;
  doubleegi_resolution_phi = M_PI / bandwidth;
  doublepcd_resolution_theta = 0.125;
  doubletan_theta_step = tan (pcd_resolution_theta * M_PI / 180);
  doublepcd_resolution_phi = 0.25;
  doubletan_phi_step = tan (pcd_resolution_phi * M_PI / 180);
  double*pcd_theta = new double[cloud->size ()];
  double*pcd_phi = new double[cloud->size ()];
  double*rho = new double[cloud->size ()];
  double*cos_angle_difference = new double[cloud->size ()];
  for (size_t i = 0; i < cloud->size (); i++)
  {
    rho[i] = sqrt (cloud->points[i].x * cloud->points[i].x +
                   cloud->points[i].y * cloud->points[i].y +
                   cloud->points[i].z * cloud->points[i].z);
    cos_angle_difference[i] = -(cloud->points[i].x * cloud_normals->points[i].normal_x +
                                cloud->points[i].y * cloud_normals->points[i].normal_y +
                                cloud->points[i].z * cloud_normals->points[i].normal_z) / rho[i];
    //std::cout << cos_angle_difference[i] << std::endl;
    pcd_theta[i] = acos (-cloud_normals->points[i].normal_z);
    pcd_phi[i] = atan2 (-cloud_normals->points[i].normal_y, -cloud_normals->points[i].normal_x);
    if (pcd_phi[i] < 0.0)
      pcd_phi[i] = pcd_phi[i] + 2 * M_PI;
  }
  doublearea = 0.0;
  int u = 0, v = 0;
  for (size_t i = 0; i < cloud_normals->size (); i++)
  {
    area = 1;//rho[i] * rho[i];// / cos_angle_difference[i];
    u = floor (pcd_theta[i] / egi_resolution_theta);
    v = floor (pcd_phi[i] / egi_resolution_phi);
    egi[u * 2 * bandwidth + v] += area;
  }

  delete [] pcd_theta;
  delete [] pcd_phi;
  delete [] rho;
  delete [] cos_angle_difference;
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
    doublemax = 0.0;
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      if (egi[i] > max)
      {
        max = egi[i];
      }
    }
    double*gray = new double[4 * bandwidth * bandwidth];
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      gray[i] = egi[i] / max;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere->resize (4 * bandwidth * bandwidth);
    uint8_t r = 0, g = 0, b = 0;
    uint32_t rgb = 0;
    doubletmp_theta = 0.0;
    doubletmp_phi = 0.0;
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
        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<double*> (&rgb);
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
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, viewports[0]);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, viewports[1]);
    viewer.addPointCloud (normals, "normals", viewports[0]);
    viewer.addPointCloud (sphere, "EGI", viewports[1]);
    //viewer.addCoordinateSystem (0.5);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normals");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "EGI");
    viewer.spin ();
  }
  return;
}

void
egiFromRange (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double*egi, const int bandwidth, const bool write2file, const bool visualization)
{
  doubleegi_resolution_theta = M_PI / bandwidth / 2;
  doubleegi_resolution_phi = M_PI / bandwidth;
  double*pcd_theta = new double[cloud->size ()];
  double*pcd_phi = new double[cloud->size ()];
  double*rho = new double[cloud->size ()];
  int *count = new int [4 * bandwidth * bandwidth];
  memset(count, 0, 4 * bandwidth * bandwidth * sizeof (int));

  for (size_t i = 0; i < cloud->size (); i++)
  {
    rho[i] = sqrt (cloud->points[i].x * cloud->points[i].x +
                   cloud->points[i].y * cloud->points[i].y +
                   cloud->points[i].z * cloud->points[i].z);
    pcd_theta[i] = acos (cloud->points[i].z / rho[i]);
    pcd_phi[i] = atan2 (cloud->points[i].y, cloud->points[i].x);
    if (pcd_phi[i] < 0.0)
      pcd_phi[i] = pcd_phi[i] + 2 * M_PI;
  }

  int u = 0, v = 0;
  for (size_t i = 0; i < cloud->size (); i++)
  {
    if (rho[i] <= 30.0)
    {
      u = floor (pcd_theta[i] / egi_resolution_theta);
      v = floor (pcd_phi[i] / egi_resolution_phi);
      egi[u * 2 * bandwidth + v] += rho[i];
      count[u * 2 * bandwidth + v] += 1;
    }
  }

  for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
  {
    if (count[i] > 1)
    {
      egi[i] = egi[i] / count[i];
    }
  }

  delete [] pcd_phi;
  delete [] pcd_theta;
  delete [] rho;
  delete [] count;

  if (write2file)
  {
    std::ofstream egi_file;
    egi_file.open ("scan001.egi");
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      egi_file << egi[i] << std::endl << "0" << std::endl;
    }
    egi_file.close ();
  }

  if (visualization)
  {
    doublemax = 0.0;
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      if (egi[i] > max)
      {
        max = egi[i];
      }
    }
    double*gray = new double[4 * bandwidth * bandwidth];
    for (size_t i = 0; i < 4 * bandwidth * bandwidth; i++)
    {
      gray[i] = egi[i] / max;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr range (new pcl::PointCloud<pcl::PointXYZ>);
    range->resize(cloud->size());
    for (size_t i = 0; i < cloud->size(); i++)
    {
      range->points[i].x = cloud->points[i].x / 30.0;
      range->points[i].y = cloud->points[i].y / 30.0;
      range->points[i].z = cloud->points[i].z / 30.0;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere->resize (4 * bandwidth * bandwidth);
    uint8_t r = 0, g = 0, b = 0;
    uint32_t rgb = 0;
    doubletmp_theta = 0.0;
    doubletmp_phi = 0.0;
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
        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<double*> (&rgb);
      }
    }
    delete [] gray;
    int viewports[2] = {1, 2};
    pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, viewports[0]);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, viewports[1]);
    viewer.addPointCloud (range, "cloud", viewports[0]);
    viewer.addPointCloud (sphere, "EGI", viewports[1]);
    viewer.addCoordinateSystem (1.0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "EGI");
    viewer.spin ();
  }
  return;
}

void softFFTWCorrelateReal(double*signal, double*pattern, const int bw, double&alpha, double&beta, double&gamma)
{
  doubletstart = csecond ();
  FILE *fp;
  softFFTWCor2( bw, signal, pattern, &alpha, &beta, &gamma, true) ;
  printf("alpha = %f\nbeta = %f\ngamma = %f\n", alpha, beta, gamma );
  doubletstop = csecond ();
  PCL_INFO ("%f seconds elapsed.\n", tstop - tstart);
  return;
}

void softFFTWCorrelate(double*sigR, double*sigI,
                       double*patternR, double*patternI,
                       const int bwIn, const int bwOut, const int degLim, bool wirte2file,
                       double&alpha, double&beta, double&gamma)
{
  doubletstart = csecond ();
  FILE *fp;
  int i;
  int n;
  double*workspace3;
  double*sigCoefR, *sigCoefI;
  double*patCoefR, *patCoefI;
  fftw_plan p1;
  int na[2], inembed[2], onembed[2];
  int rank, howmany, istride, idist, ostride, odist;
  int tmp, maxloc, ii, jj, kk;
  doublemaxval, tmpval;
  double*weights;
  double*seminaive_naive_tablespace;
  double**seminaive_naive_table;
  fftw_plan dctPlan, fftPlan;
  int howmany_rank;
  fftw_iodim dims[1], howmany_dims[1];

  n = 2 * bwIn;
  fftw_complex *so3Sig = (fftw_complex *)fftw_malloc (sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
  fftw_complex *workspace1 = (fftw_complex *)fftw_malloc (sizeof(fftw_complex) * (8 * bwOut * bwOut * bwOut));
  fftw_complex *workspace2 = (fftw_complex *)fftw_malloc (sizeof(fftw_complex) * ((14 * bwIn * bwIn) + (48 * bwIn)));
  workspace3 = (double*)malloc (sizeof(double) * (12 * n + n * bwIn));
  sigCoefR = (double*)malloc (sizeof(double) * bwIn * bwIn);
  sigCoefI = (double*)malloc (sizeof(double) * bwIn * bwIn);
  patCoefR = (double*)malloc (sizeof(double) * bwIn * bwIn);
  patCoefI = (double*)malloc (sizeof(double) * bwIn * bwIn);
  fftw_complex *so3Coef = (fftw_complex *)fftw_malloc (sizeof(fftw_complex) * ((4 * bwOut * bwOut * bwOut - bwOut) / 3));

  seminaive_naive_tablespace = (double*)malloc (sizeof(double) * (Reduced_Naive_TableSize (bwIn, bwIn)
      + Reduced_SpharmonicTableSize (bwIn, bwIn)));

  weights = (double*)malloc (sizeof(double) * (4 * bwIn));

  /****
   At this point, check to see if all the memory has been
   allocated. If it has not, there's no point in going further.
   ****/

  if ((seminaive_naive_tablespace == NULL) || (weights == NULL) ||
      (sigR == NULL) || (sigI == NULL) || (patternR == NULL) || (patternI == NULL) ||
      (so3Coef == NULL) || (workspace1 == NULL) || (workspace2 == NULL) || (workspace3 == NULL) ||
      (sigCoefR == NULL) || (sigCoefI == NULL) || (patCoefR == NULL) || (patCoefI == NULL) || (so3Sig == NULL))
  {
    perror ("Error in allocating memory");
    exit (1);
  }

  /* create fftw plans for the S^2 transforms */
  /* first for the dct */
  dctPlan = fftw_plan_r2r_1d (2 * bwIn, weights, workspace3, FFTW_REDFT10, FFTW_ESTIMATE);

  /* now for the fft */
  /*
   IMPORTANT NOTE!!! READ THIS!!!
   Now to make the fft plans.
   Please note that the planning-rigor flag *must be* FFTW_ESTIMATE! Why? Well, to try to keep things simple.
   I am using some of the pointers to arrays in rotateFct's arguments in the fftw-planning routines. If the
   planning-rigor is *not* FFTW_ESTIMATE, then the arrays will be written over during the planning stage.
   Therefore, unless you are really really sure you know what you're doing, keep the rigor as FFTW_ESTIMATE !!!
   */
  /*
   fftw "preamble" ; note  that this places in the transposed array
   */

  rank = 1;
  dims[0].n = 2 * bwIn;
  dims[0].is = 1;
  dims[0].os = 2 * bwIn;
  howmany_rank = 1;
  howmany_dims[0].n = 2 * bwIn;
  howmany_dims[0].is = 2 * bwIn;
  howmany_dims[0].os = 1;

  fftPlan = fftw_plan_guru_split_dft (rank, dims, howmany_rank, howmany_dims, sigR, sigI, (double*)workspace2,
                                      (double*)workspace2 + (n * n), FFTW_ESTIMATE);

  /* create plan for inverse SO(3) transform */
  n = 2 * bwOut;
  howmany = n * n;
  idist = n;
  odist = n;
  rank = 2;
  inembed[0] = n;
  inembed[1] = n * n;
  onembed[0] = n;
  onembed[1] = n * n;
  istride = 1;
  ostride = 1;
  na[0] = 1;
  na[1] = n;

  p1 = fftw_plan_many_dft (rank, na, howmany, workspace1, inembed, istride, idist, so3Sig, onembed, ostride, odist,
                           FFTW_FORWARD, FFTW_ESTIMATE);

  fprintf (stdout, "Generating seminaive_naive tables...\n");
  seminaive_naive_table = SemiNaive_Naive_Pml_Table (bwIn, bwIn, seminaive_naive_tablespace, (double*)workspace2);

  /* make quadrature weights for the S^2 transform */
  makeweights (bwIn, weights);

  n = 2 * bwIn;

  printf ("now taking spherical transform of signal\n");
  FST_semi_memo (sigR, sigI, sigCoefR, sigCoefI, bwIn, seminaive_naive_table, (double*)workspace2, 0, bwIn, &dctPlan,
                 &fftPlan, weights);


  printf ("now taking spherical transform of pattern\n");
  FST_semi_memo (patternR, patternI, patCoefR, patCoefI, bwIn, seminaive_naive_table, (double*)workspace2, 0, bwIn, &dctPlan,
                 &fftPlan, weights);

  printf ("freeing seminaive_naive_table and seminaive_naive_tablespace\n");

  free (seminaive_naive_table);
  free (seminaive_naive_tablespace);

  printf ("about to combine coefficients\n");

  /* combine coefficients */

  so3CombineCoef_fftw (bwIn, bwOut, degLim, sigCoefR, sigCoefI, patCoefR, patCoefI, so3Coef);


  printf ("about to inverse so(3) transform\n");
  /* now inverse so(3) */
  Inverse_SO3_Naive_fftw (bwOut, so3Coef, so3Sig, workspace1, workspace2, workspace3, &p1, 0);
  printf ("finished inverse so(3) transform\n");

  /* now find max value */
  maxval = 0.0;
  maxloc = 0;
  for (i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
  {
    tmpval = NORM (so3Sig[i]);
    if (tmpval > maxval)
    {
      maxval = tmpval;
      maxloc = i;
    }
  }

  ii = floor (maxloc / (4. * bwOut * bwOut));
  tmp = maxloc - (ii * 4. * bwOut * bwOut);
  jj = floor (tmp / (2. * bwOut));
  tmp = maxloc - (ii * 4 * bwOut * bwOut) - jj * (2 * bwOut);
  kk = tmp;

  printf ("ii = %d\tjj = %d\tkk = %d\n", ii, jj, kk);
  printf ("alpha = %f\nbeta = %f\ngamma = %f\n", M_PI * jj / ((double)bwOut), M_PI * (2 * ii + 1) / (4. * bwOut), M_PI
      * kk / ((double)bwOut));
  alpha = M_PI * jj / ((double)bwOut);
  beta = M_PI * (2 * ii + 1) / (4. * bwOut);
  gamma = M_PI * kk / ((double)bwOut);
  /* now save data -> just the real part because the
   imaginary parts should all be 0 ... right?!? */
//  if (argc == 7)
//  {
//    printf ("about to save data\n");
//    fp = fopen (argv[6], "w");
//    for (i = 0; i < 8 * bwOut * bwOut * bwOut; i++)
//      fprintf (fp, "%.16f\n", so3Sig[i][0]);
//    fclose (fp);
//  }

  fftw_destroy_plan (p1);
  fftw_destroy_plan (fftPlan);
  fftw_destroy_plan (dctPlan);

  free (weights);
  fftw_free (so3Coef);
  free (patCoefI);
  free (patCoefR);
  free (sigCoefI);
  free (sigCoefR);
  free (workspace3);
  fftw_free (workspace2);
  fftw_free (workspace1);
  fftw_free (so3Sig);

  doubletstop = csecond ();
  PCL_INFO ("%f seconds elapsed.\n", tstop - tstart);
  return;
}

void
heatmapRGB (doublegray, uint8_t &r, uint8_t &g, uint8_t &b)
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

void rotationMatrixFromEulerAngles(double*rm, const doublealpha, const doublebeta, const doublegamma)
{
  doublecalpha = cos(alpha);
  doublesalpha = sin(alpha);
  doublecbeta = cos(beta);
  doublesbeta = sin(beta);
  doublecgamma = cos(gamma);
  doublesgamma = sin(gamma);
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
rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                 double*rm)
{
  output->resize(input->size());
  for (size_t i = 0; i < input->size(); i++)
  {
    output->points[i].x = rm[0] * input->points[i].x + rm[1] * input->points[i].y + rm[2] * input->points[i].z;
    output->points[i].y = rm[3] * input->points[i].x + rm[4] * input->points[i].y + rm[5] * input->points[i].z;
    output->points[i].z = rm[6] * input->points[i].x + rm[7] * input->points[i].y + rm[8] * input->points[i].z;
  }
}
