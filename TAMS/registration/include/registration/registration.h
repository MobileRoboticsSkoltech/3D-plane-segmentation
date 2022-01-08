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

#ifndef TAMS_REGISTRATION_H_
#define TAMS_REGISTRATION_H_

//STL
#include <string>
#include <cmath>
#include <fstream>
//#include <unordered_set>
//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/surface/mls.h>
//boost
#include <boost/timer.hpp>
#include <boost/unordered_set.hpp>
#include <boost/functional/hash.hpp>
//Eigen
#include <Eigen/Householder>
#include <Eigen/QR>
//tams
#include "common/planar_patch.h"
#include "registration_parameters.h"
#include "abstract_planar_segment/abstract_planar_segment.h"

namespace tams
{
  /** stands for area-consistent planar patch pairs.*/
  struct AreaConsistentPair
  {
    AreaConsistentPair(){}
    AreaConsistentPair(int _lhs,
                       int _rhs)
    {
      lhs = _lhs;
      rhs = _rhs;
    }
    int lhs;
    int rhs;

    typedef std::vector<AreaConsistentPair> StdVector;
    typedef boost::shared_ptr<StdVector> StdVectorPtr;
  };



  class AreaConsistentPairTriplets
  {
    public:
      AreaConsistentPairTriplets(int a , int b, int c):
         first(a), second (b), third (c)
      {

      }
      AreaConsistentPairTriplets():
        first(-1), second(-1), third(-1)
      {
      }

      int first;
      int second;
      int third;

      friend bool
      operator== (const AreaConsistentPairTriplets &x, const AreaConsistentPairTriplets &y)
      {
        return (x.first == y.first && x.second == y.second && x.third == y.third);
      }

      friend std::size_t hash_value(const AreaConsistentPairTriplets &x)
      {  return std::size_t(x.first * 100 + x.second * 10 + x.third); }
  };

  struct Solution
  {
    Vector3d translation;
    Matrix3d rotation;
    double total_area;
    AreaConsistentPair::StdVector correspondences;
  };

  struct RCPPPair
  {
    RCPPPair(){}
    RCPPPair(int _lhs_1,
             int _lhs_2,
             int _rhs_1,
             int _rhs_2,
             double _probability)
  {
      lhs_1 = _lhs_1;
      lhs_2 = _lhs_2;
      rhs_1 = _rhs_1;
      rhs_2 = _rhs_2;
      probability = _probability;
  }
    int lhs_1;//the index of the first plane in the left hand side point cloud
    int lhs_2;//the index of the second plane in the left hand point cloud
    int rhs_1;//the index of the first plane in the right hand side point cloud
    int rhs_2;//the index of the second plane in the right hand side point cloud
    double probability;
    typedef boost::shared_ptr<RCPPPair> Ptr;
    typedef std::vector<RCPPPair> StdVector;
    typedef boost::shared_ptr<StdVector> StdVectorPtr;
  };
/** \brief @b Registration represents 3-D point cloud registration based on planar segments.
  * Given two cloud points, this class estimation their relative rotation and translation.
  * \author Junhao Xiao
  * \ingroup Registration
  */
class Registration
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

  typedef Eigen::Vector3d Vector3d;
  typedef Eigen::Matrix3d Matrix3d;
  typedef Eigen::Vector4d Vector4d;
  typedef Eigen::Matrix4d Matrix4d;
  typedef Eigen::Matrix<double, 5, 1> Vector5d;
  typedef Eigen::Matrix<double, 5, 5> Matrix5d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  public:
  /** \brief empty construction.*/
  Registration():
    map_cloud_ (new PointCloud), data_cloud_ (new PointCloud),
    transformed_data_cloud_ (new PointCloud), rotated_data_cloud_ (new PointCloud),
    translated_data_cloud_ (new PointCloud),
    map_vis_ (new pcl::PointCloud<pcl::PointXYZRGB>), data_vis_ (new pcl::PointCloud<pcl::PointXYZRGB>),
    map_segments_ (new PlanarSegment::StdVector), data_segments_ (new PlanarSegment::StdVector),
    big_map_segments_(new PlanarSegment::StdVector), big_data_segments_(new PlanarSegment::StdVector),
    area_consistent_planes_(new AreaConsistentPair::StdVector), rotation_consistent_pairs_(new RCPPPair::StdVector),
    rotation_ (Matrix3d::Zero()), translation_ (Vector3d::Zero())
  {
  }
  /** \brief empty destruction*/
  ~Registration()
  {

  }

  /** \brief Set the cloud which will be used as a reference map.
   * \param[in] cloud pointer of the reference cloud
   */
  void
  setMapCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    map_cloud_ = cloud;
  }

  /** \brief Set the cloud whose rotation will be aligned to the map
   * \param [in] cloud  pointer of the given cloud
   */
  void
  setDataCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    data_cloud_ = cloud;
  }

  /**
   * @b Only the top N segments (with regards to area) will be used for registration.
   * @param[in] N the segments number used for registration
   */
  void
  getBigSegments(int N);

  /**
   * @b Only the segments with bigger area than min_area will be used for registration
   * @param[in] min_area threshold for filtering segments
   */
  void
  getBigSegments(double min_area);

  /**
   * Get the planar patches from a planar segmentation of the given map point cloud.
   * @param[in] segments a boost shared_ptr point to a vector of planar patches.
   */
  void
  setMapSegments(PlanarSegment::StdVectorPtr segments);

  /**
   * @b read planar segments as map from from a given file, where stores the planar patches
   * resulted from planar segmentation
   * @param[in] segments_file the name of the text file
   */
  void
  setMapSegments(std::string segments_file);

  /**
   * Get the planar patches from a planar segmentation of the given data point cloud.
   * @param[in] segments a boost shared_ptr point to a vector of planar patches.
   */
  void
  setDataSegments(PlanarSegment::StdVectorPtr segments);

  /**
   * read planar segments as map from a given file, where stores the planar patches
   * resulted from planar segmentation
   * @param[in] segments_file the name of the text file
   */
  void
  setDataSegments(std::string segments_file);

  /**
   * @b Rotate a given point cloud with given rotation matrix in SO(3).
   * @param[in] input boost shared pointer to the given point cloud which will be rotated
   * @param[out] output boost shared pointer to the output (rotated) point cloud
   * @param[in] rotation matrix
   */
  void
  rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                   Matrix3d rotation);


  /**
   * Transform a given point cloud using the given 3-D rotation matrix and translation vector.
   * @param[in] input boost shared pointer to the given point cloud which will be transformed
   * @param[out] outpu boost shared pointer to the output (transformed) point cloud
   * @param[in] rotation rotation matrix
   * @param[in] translation translation vector
   */


  void
  transformPointcloud(PointCloudPtr input,
                      PointCloudPtr output,
                      Matrix3d rotation,
                      Vector3d trans);

  /**
   * @b Visualize the correspondence plane pairs.
   */
  void visualization();

  void
  colorEncoding (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
                 PlanarSegment::StdVectorPtr segments);

  /**
   * @b Merge the planar patches which are on the same optimal infinite plane.
   */
  void
  mergeSurfacesOnSameInfinitePlane(double min_dot_product,
                                   double max_bias_dif);



  void
  execute();

  Vector3d
  translation(){return translation_;}
  Matrix3d
  rotation () {return rotation_;}

  /**
   * @b Set the parameters for the registration algorithm.
   * @params[in] param registration parameters
   */
  void
  setParameters(RegistrationParameters params)
  {
    params_=params;
    params_.max_angle_diff = params_.max_angle_diff * M_PI / 180;
    params_.unparallel_max_angle = params_.unparallel_max_angle * M_PI / 180;
    params_.unparallel_min_angle = params_.unparallel_min_angle * M_PI / 180;
    params_.side_rotation_ability = params_.side_rotation_ability * M_PI / 180;
    params_.merge_angle = params_.merge_angle * M_PI / 180;

  }

  /** @b Visualize the planar surface correspondences between the two given point clouds.
   *
   */
  void
  visualizeCorrespondences();

  void
  visualizeCorrespondencesWithPoints();

private:
  /**
   * @b Find area consistent planes. Suppose \f$p_l\f$ is a plane from the
   * left hand side point cloud, and \f$p_r\f$ is a plane from the right
   * hand side point cloud, they are considered as area-consistent
   * if \f$(p_l.s-p_r.s)/max(p_l.s, p_r.s)<max_dif\f$
   * @param[in] max_dif the threshold for determine area consistent.
   */
  void
  findAreaConsistentPlanes(double max_dif);

  /** filter out planar segment with almost linear shape. */
  bool
  filterByLinearity(double value);

  /** @ Find all possible 3 non-parallel planar segment triplets.*/
  void
  findAreaConsistentPairTriplets(double simple_translation_test_threshold,
                                 double min_angle,
                                 double max_angle);

  /** @b Find all potential solutions according to a minmum required concensus set size. */
  void
  findPotentialSolutions();

  /** \brief Guarantee the correspondences are unique, in other words, there is no segment which corresponds to multiple segments.
    *@param[in] solution the solution whose correspondences will be checked if unique.*/
  void
  ensureUniqueMapping(Solution &solution);


//  void
//  mainloop(double simple_translation_test_threshold,
//                         double min_angle,
//                         double max_angle);


  /** \brief A simple overlapping test using area, rotation and translation.
    * If two segements p_l, p_r overlapped each other after a transformation (rotation + translation),
    * they should fulfill
    * (rotation_l_r * p_r.mass + translation_l_r - p_l.mass).norm() < sqrt(p_l.area/M_PI) + sqrt(p_r.area/M_PI)
    * \param one_pair the first correspondence segment pair
    * \param rotation 3X3 rotation matrix
    * \param translation 3X1 translation vector
    */
  bool
  overlapping (AreaConsistentPair one_pair,
               Matrix3d rotation,
               Vector3d translation);



  Vector3d
  translationFromThreeCorrespondencePairs(AreaConsistentPair::StdVector::iterator it1,
                                          AreaConsistentPair::StdVector::iterator it2,
                                          AreaConsistentPair::StdVector::iterator it3);

  Vector3d
  translationFromThreeCorrespondencePairs(AreaConsistentPair pair1,
                                          AreaConsistentPair pair2,
                                          AreaConsistentPair pair3);

  Matrix3d
  rotationFromTwoCorrespondencePairs(AreaConsistentPair::StdVector::iterator it1,
                                     AreaConsistentPair::StdVector::iterator it2);
  Matrix3d
  rotationFromTwoCorrespondencePairs(AreaConsistentPair pair1, AreaConsistentPair pair2);

  /**
   * @b Refine the rotation of a solution using least squares based on SVD.
   * @param[in] solution the given solution with plane correspondences.
   * @param[in] value the difference between the translation found by initial three-non-planar-segments and found by all
   *            correspondences should not be larger than this threshold
   */
  bool
  findRotation(Solution solution, double value);

  /**
   * @b Refine the translation of a solution using least squares based on ColPivHouseholderQR.
   * @param[in] solution the given solution with plane correspondences
   * @param[in] value the difference between the translation found by initial three-non-planar-segments and found by all
   *             correspondences should not be larger than this threshold
   */
  bool
  findTranslation(Solution solution, double value);


  /** @b The initial rotation and translation was computed by freezing two and three correspondences, respectively.
   * They will be refined using all the consistent correspondences to the initial rotation and translation.
   */
  void
  refineSolutions();

  /** @b Find the optimum solution from potential solutions.
   *  The main metric is the shperical correlation and number of corresondences segment pairs.
      */
  bool
  findOptimumSolution();

  void
  point2plane(Solution solution);

  bool
  exceedLocomotionAbility(Matrix3d rotation);

  bool
  exceedLocomotionAbility(Vector3d translation);

private:
  PointCloudPtr map_cloud_;
  PointCloudPtr data_cloud_;
  PointCloudPtr transformed_data_cloud_;
  PointCloudPtr rotated_data_cloud_;
  PointCloudPtr translated_data_cloud_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_vis_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_vis_;
  PlanarSegment::StdVectorPtr map_segments_;
  PlanarSegment::StdVectorPtr data_segments_;
  PlanarSegment::StdVectorPtr big_map_segments_;
  PlanarSegment::StdVectorPtr big_data_segments_;
  AreaConsistentPair::StdVectorPtr area_consistent_planes_;
  RCPPPair::StdVectorPtr rotation_consistent_pairs_;
  AreaConsistentPair::StdVector single_rotation_consistents_;
  AreaConsistentPair::StdVector single_translation_consitents_;
  std::vector<AreaConsistentPair::StdVector> all_rotation_consistents_;
  std::vector<AreaConsistentPair::StdVector> all_translation_consistents_;
  std::vector<Solution> solutions_;
  boost::unordered_set<AreaConsistentPairTriplets> area_consistent_pair_triplets_;
  double alpha_;
  double beta_;
  double gamma_;
  Matrix3d rotation_;
  Vector3d translation_;
  std::vector<Matrix3d, aligned_allocator<Matrix3d> > rotations_;
  RegistrationParameters params_;

  /** for verbose output. */
  bool DEBUG;
};
}
#endif
