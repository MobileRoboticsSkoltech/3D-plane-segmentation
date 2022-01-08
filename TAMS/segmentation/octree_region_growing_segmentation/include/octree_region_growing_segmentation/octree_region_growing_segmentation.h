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

#ifndef OCTREE_REGION_GROWING_SEGMENTATION_H_
#define OCTREE_REGION_GROWING_SEGMENTATION_H_
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include "pcl/search/pcl_search.h"
#include <Eigen/Eigenvalues>

#include "common/planar_patch.h"
#include "common/sensor_parameters.h"
#include "octree_region_growing_segmentation_parameters.h"

#include <iostream>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <utility>
namespace tams
{
  using namespace std;
  class SlidingSphreItem
  {
  public:
    int index;
    double mse;
    Vector3d normal;
  public:
    SlidingSphreItem () :
      index (0), mse (0), normal (Vector3d::Zero())
    {
    }
    ~SlidingSphreItem ()
    {
    }
    bool
    operator< (const SlidingSphreItem& rhs) const
    {
      return mse < rhs.mse;
    }
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class OctreeRGSegmentation
  {
    public:
      typedef boost::shared_ptr<OctreeRGSegmentation> Ptr;
  public:
    /** \brief Empty constructor.*/
    OctreeRGSegmentation():
      input_ (new pcl::PointCloud<pcl::PointXYZ>), cloud_ (new pcl::PointCloud<pcl::PointXYZ>),
      max_neighbor_dis_(0.0), max_point2plane_dis_(0.0), max_angle_difference_ (0.0), max_segment_mse_(0.0),
      max_local_mse_ (0.0), max_seed_mse_ (0.0), nearest_neighbor_size_ (0), min_segment_size_(0.0),
      sliding_sphere_size_ (0), pcd_size_(0), downsampling_ (false),show_filtered_cloud_ (false),
      downsampling_leafsize_ (0.0f), osr_mean_k_ (0), osr_StddevMulThresh_ (0.0f),
      planar_patches_ (new PlanarSegment::StdVector), visited_ (NULL), added_to_region_ (NULL),
      has_local_plane_ (NULL), local_mse_ (NULL), nn_indices_ (NULL), nn_dis_ (NULL),
      badpoints_num_ (0)
    {

    }


    /** \brief Fitting a plane to each point with its neighbours, where local normal
      * and local mse are also computed.*/
    void
    slidingSphere();

    /** \brief Caching the neighbour indices in an array for each point in the cloud.
     */
    void
    octreeCaching();

    /** \brief Segment the input point cloud into planar patches.
     */
    void segmentation();

    /** \brief Set pre-tuned thresholds for the algorithm. */
    void setParameters (OctreeRegionGrowingSegmentationParameters &parameters);

//    /** \brief Set the noisy model of the corresponding range sensor.
//        @param poly[3] coefficients of the second-order polynomial.
//      */
//    void setSensorNoiseModel(const double a0, const double a1, const double a2);

    /** \brief Set the input cloud which can be organized or disorganized. */
    void setInput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    /** \brief Get the boost shared point to the cloud, in case downsampling happened. */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud()
    {
      return cloud_;
    }

    /** \brief Downsample the given cloud, this however only works for disorganized
      * cloud. Since the indices mapping relation will be lost after downsampling,
      * the mapping relation will be used further for example segment area calculation,
      * see class SegmentsArea for detail.
      */
    void downsampling ();

    PlanarSegment::StdVectorPtr
    getSegments()
    {
      return planar_patches_;
    }

    void
    segmentsIntensityHistogram();

    /** \brief Empty deconstructor.*/
    ~OctreeRGSegmentation(){}



  private:

    /** \brief Investigating the neighbours of the currently added point. A neibhour will be added
      * to the queue if it fulfills
      * a. it has not been investigated;
      * b. it has not been identified;
      * c. the distance between it and the currently added point is smaller than a pre-defined threshold.
      */
    void investigateNeighbors (int index);

    /** \brief The main body of segmentation, it will be called by
      * the function segmentation().
     */
    void
    applySegmentation ();


  private:
    /** \brief The segmentation name. */
    pcl::search::Search<pcl::PointXYZ>::Ptr tree_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    /** \brief whether the given point cloud is organized. */
    bool organized_;
    /** \brief the points in Eigen data type. */
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points_;

    double max_neighbor_dis_;
    double max_point2plane_dis_;
    double max_angle_difference_;
    double max_segment_mse_;
    double max_local_mse_;
    double max_seed_mse_;
    /** sigma =  polynomial_noise[0] + polynomial_noise * range + polynomial_noise * range * range */
    double polynomial_noise[3];
    int nearest_neighbor_size_;
    int min_segment_size_;
    int sliding_sphere_size_;
    int pcd_size_;
    bool downsampling_;
    bool show_filtered_cloud_;
    double downsampling_leafsize_;
    int osr_mean_k_;
    double osr_StddevMulThresh_;
    vector<int> neighbor_points_;
    vector<int> remained_points_;
    vector<int> uognzd_indice_to_ognzd_;
    PlanarSegment :: StdVectorPtr planar_patches_;
    bool *visited_;
    bool *added_to_region_;
    bool *has_local_plane_;
    double *local_mse_;
    int *nn_indices_;
    double *nn_dis_;
    vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > local_normals_;
    vector<SlidingSphreItem, Eigen::aligned_allocator<SlidingSphreItem> >sliding_spheres_;
    int badpoints_num_;
    ofstream ofile_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
