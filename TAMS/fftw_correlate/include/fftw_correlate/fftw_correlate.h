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

#ifndef FFTW_CORRELATE_H_
#define FFTW_CORRELATE_H_

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
#include "fftw_correlate/softFFTWCorrelateReal.h"
}

/** \brief @b FFTWCorrelate represents the relative rotation estimation class.
  * Given two cloud points, this class estimation their relative rotation as Euler angles.
  * \author Junhao Xiao
  * \ingroup Registration
  */

class FFTWCorrelate
{
public:
  /** \brief empty construction.*/
  FFTWCorrelate():
    bwIn_(0), bwOut_(0), bwLimit_(0),
    cloud_map_ (new pcl::PointCloud<pcl::PointXYZ>),
    cloud_data_ (new pcl::PointCloud<pcl::PointXYZ>),
    rotated_cloud_data_ (new pcl::PointCloud<pcl::PointXYZ>),
    rm_(new float [9])
  {
  }
  /** \brief empty destruction*/
  ~FFTWCorrelate()
  {
    if (rm_ != NULL)
      delete [] rm_;
    if (egi_map_ != NULL)
      delete [] egi_map_;
    if (egi_data_ != NULL)
      delete [] egi_data_;
    if (rms_ != NULL)
      delete [] rms_;
  }

  /** \brief Set the cloud which will be used as a reference map.
   * \param[in] cloud pointer of the reference cloud
   */
  void
  setMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    cloud_map_ = cloud;
  }

  /** \brief Set the cloud whose rotation will be aligned to the map
   * \param [in] cloud  pointer of the given cloud
   */
  void
  setData(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    cloud_data_ = cloud;
  }

  /** \brief bandwidth settings.
   * \param[in] bwIn bandwidth for the forward spherical harmonics transform and spherical Fourier Transform
   * \param[in] bwOut bandwidth for the inverse spherical fourier transform.
   * \param[in] bwLimit the limitation bandwidth of inverse spherical transform.
   */
  void
  setBandWidth(const int bwIn, const int bwOut, const int bwLimit)
  {
    bwIn_ = bwIn;
    bwOut_ = bwOut;
    bwLimit_ = bwLimit;
  }

  /** \brief Get the rotation (as Euler angles) between two clouds with overlapping. */
  void
  softFFTWCorrelateReal();

  /**
   * @b Construct EGI from octree planar segmentation or planar segmentation.
   * @param segments_file
   */
  void
  mapConstellationImage(const std::string segments_file)
  {
    constellationImage(segments_file, bwIn_, egi_map_);
  }

  /**
   * @b Construct EGI from octree planar segmentation or planar segmentation.
   * @param segments_file
   */
  void
  dataConstellationImage(const std::string segments_file)
  {
    constellationImage(segments_file, bwIn_, egi_data_);
  }
  /** \brief construct the constellation Images for map and data cloud.
   * */
  void
  constellationImage(const std::string segments_file,
                     const int bandwidth,
                     float *egi);

  /** \brief construct Extended Gaussian Image for the cloud whose coordinate system as reference.*/
  void
  egiMap (bool organized,
          bool write2file,
          bool visualization);

  /** \brief construct Extended Gaussian Image for the cloud which is to be registrated.*/
  void
  egiData (bool organized,
           bool write2file,
           bool visualization);

  /** \brief construct Extended Gaussian Image for given point cloud and bandwidth
   * (nyquist sampling on latitude and longitude)
   * \param[in] cloud boost shared pointer to the given point cloud\
   * \param[out] calculated Extended Gaussian Image.
   * \param[in] bandwidth for forward spherical harmonics transform
   * \param[in] organized whether the given point cloud is organized,
   *            octree data structure will be used for knn search if not,
   *            otherwise the pixel information will be used.
   * \param[in] write2file whether write the Extended Gaussian Image to a text file\
   * \param[in] visualization whether to visualize the Extended Gaussian Image
   */
  void
  egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                float *egi,
                const int bandwidth,
                const bool organized,
                const bool write2file,
                const bool visualization);
  /**@b Extended Gaussian Image computation for organized point cloud.
   *
   * @param[in] cloud boost shared pointer to the given organized point cloud
   * @param[out] egi pointer to the resulted Extended Gaussian Image
   * @param[in] bandwidth for forward spherical harmonics transform
   * @param[in] visualization whether to visulize the resulted Extended Gaussian Image
   */
  void
  egiFromNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                float *egi,
                const int bandwidth,
                const bool visualization);
  /**
   * @b Rotate a given point cloud with given rotation matrix in SO(3).
   * @param[in] input boost shared pointer to the given point cloud which will be rotated
   * @param[out] output boost shared pointer to the output (rotated) point cloud
   * @param[in] rm pointer to the rotation matrix (float [9])
   */
  void
  rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                   float *rm);

  void
  heatmapRGB (float gray,
              uint8_t &r,
              uint8_t &g,
              uint8_t &b);

  /**
   * @b Construct a 3D rotation matrix from given Euler angles (in z-y-z turn).
   * @param[out] rm resulted rotation matrix
   * @param[in] alpha the first Euler angle
   * @param[in] beta the second Euler angle
   * @param[in] gamma the third Euler angle
   */
  void
  rotationMatrixFromEulerAngles(float *rm,
                                const float alpha,
                                const float beta,
                                const float gamma);

  /**
   * @b construct the rotation matrix table.
   */
  void
  initialize ();



  /**
   * @b visualization
   */
  void
  visualize();

  /**
   * @b step to next point cloud.
   */
  void
  dataAsMap ();

  /**
   * @b tmp
   */
  void test ();

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud_data_;
  int bwIn_;
  int bwOut_;
  int bwLimit_;
  float alpha_;
  float beta_;
  float gamma_;
  float *rm_;
  float *egi_map_;
  float *egi_data_;
  float *rms_;
};

#endif
