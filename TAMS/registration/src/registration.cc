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
#include <algorithm>
#include <fstream>
//Eigen
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Geometry>
//PCL
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
//TAMS
#include "registration/registration.h"
#include "common/rgb.h"
#include "common/common.h"
#include "common/planar_patch.h"

using namespace tams;
void
Registration::colorEncoding (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
                             PlanarSegment::StdVectorPtr segments)
{

  std::vector<RGB> colors;
  getColors(colors);
  int point_num = 0;
  for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
  {
    point_num += it->point_num;
  }
  output->points.clear();
  output->points.resize(point_num);
  output->height = 1;
  output->width = point_num;
  output->resize(point_num);
  int cnt = 0;
  for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
  {
    int gray = 0;
    int color_index;
    while (gray<75)
    {
      color_index = rand() % colors.size();
      gray = (colors[color_index].r + colors[color_index].g + colors[color_index].b) / 3;
    }
    int32_t rgb_integer = (colors[color_index].r << 16) | (colors[color_index].g << 8) | colors[color_index].b;
    double rgb = *(double *)(&rgb_integer);
    for (int j = 0; j < it->points.size(); j++)
    {
      int index = it->points[j];
      output->points[cnt].x = cloud->points[index].x;
      output->points[cnt].y = cloud->points[index].y;
      output->points[cnt].z = cloud->points[index].z;
      output->points[cnt].rgb = rgb;
      cnt++;
    }
  }
}

bool
Registration::exceedLocomotionAbility(Matrix3d rotation)
{
  Vector3d z = Vector3d::UnitZ();
  Vector3d rotated_z = rotation*z;
  double angle = acos(z.dot(rotated_z));
  if (angle > params_.side_rotation_ability)
  {
    PCL_DEBUG ("Angle = %f, which exceeds the locomotion ability!\n", angle);
    return true;
  }

  return false;
}

bool
Registration::exceedLocomotionAbility(Vector3d translation)
{
  if (translation.norm() > params_.max_translation_norm)
  {
    //PCL_INFO ("translation = %f, which exceeds the biggest translation between two scan points!\n", translation.norm());
    return true;
  }

  double angle = atan(translation(2) / sqrt(translation(0) * translation(0) + translation(1) * translation(1)));
  if (fabs(angle) > params_.side_rotation_ability)
  {
    //PCL_INFO ("Angle = %f, which exceeds the locomotion ability!\n", angle);
    return true;
  }

  return false;
}


void
Registration::findAreaConsistentPlanes(double max_dif)
{
  area_consistent_planes_->clear();
  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;
  double dif;
  for (map_it = big_map_segments_->begin(); map_it != big_map_segments_->end(); map_it++)
  {
    for (data_it = big_data_segments_->begin(); data_it != big_data_segments_->end(); data_it++)
    {
      dif = fabs(map_it->area - data_it->area)/std::max(map_it->area, data_it->area);
      //dif = 2.0f * fabs(map_it->area - data_it->area)/(map_it->area + data_it->area);
      if (dif < max_dif)
      {
        area_consistent_planes_->push_back(AreaConsistentPair(map_it - big_map_segments_->begin(), data_it - big_data_segments_->begin()));
      }
    }
  }
  PCL_INFO ("%d area consistent planes have been found.\n", area_consistent_planes_->size());
}


bool
Registration::filterByLinearity (double value)
{
  Matrix3d scatter_matrix;
  Eigen::EigenSolver<Eigen::Matrix3d> eigensolver3d;
  Vector3d eigenvalues3d = Vector3d::Zero();
  Matrix3d eigenvectors3d = Matrix3d::Zero();
  int min_eigenvalue_index;
  double min_eigenvalue;
  double linearity;
  for (PlanarSegment::StdVector::iterator it = big_map_segments_->begin(); it != big_map_segments_->end() ; it++)
  {
    scatter_matrix = it->scatter_matrix;
    eigensolver3d.compute(scatter_matrix);
    eigenvalues3d = eigensolver3d.eigenvalues().real();
    eigenvectors3d = eigensolver3d.eigenvectors().real();
    min_eigenvalue = eigenvalues3d.minCoeff(&min_eigenvalue_index);

    double lambada[2];
    int i = 0;
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
    if (linearity > value)
    {
      it->area = 0.0;
    }
  }

  PlanarSegment::StdVector segments;
  for (PlanarSegment::StdVector::iterator it = big_map_segments_->begin(); it != big_map_segments_->end() ; it++)
  {
    if (it->area == 0.0)
    {
      continue;
    }
    segments.push_back (*it);
  }

  big_map_segments_->clear ();
  big_map_segments_->insert (big_map_segments_->begin (), segments.begin (), segments.end ());

  for (PlanarSegment::StdVector::iterator it = big_data_segments_->begin(); it != big_data_segments_->end() ; it++)
  {
    scatter_matrix = it->scatter_matrix;
    eigensolver3d.compute(scatter_matrix);
    eigenvalues3d = eigensolver3d.eigenvalues().real();
    eigenvectors3d = eigensolver3d.eigenvectors().real();
    min_eigenvalue = eigenvalues3d.minCoeff(&min_eigenvalue_index);

    double lambada[2];
    int i = 0;
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
    if (linearity > value)
    {
      it->area = 0.0;
    }
  }

  segments.clear ();
  for (PlanarSegment::StdVector::iterator it = big_data_segments_->begin(); it != big_data_segments_->end() ; it++)
  {
    if (it->area == 0.0)
    {
      continue;
    }
    segments.push_back (*it);
  }

  big_data_segments_->clear ();
  big_data_segments_->insert (big_data_segments_->begin (), segments.begin (), segments.end ());
}

void
Registration::getBigSegments(double min_area)
{
  PlanarSegment pp;
  PlanarSegment::StdVector::iterator it;
  big_map_segments_->clear();
  big_data_segments_->clear();
  for (it = map_segments_->begin(); it != map_segments_->end() ; it++)
  {
    if (it->area > min_area)
      big_map_segments_->push_back(*it);
  }
  for (it = data_segments_->begin(); it != data_segments_->end() ; it++)
  {
    if (it->area > min_area)
      big_data_segments_->push_back(*it);
  }
  PCL_INFO ("There are %d and %d segments with area bigger than %f in the map cloud and data cloud respectively.\n",
            big_map_segments_->size(), big_data_segments_->size(), min_area);
}


void
Registration::getBigSegments(int N)
{
  PlanarSegment pp;
  PlanarSegment::StdVector::iterator it_i;
  PlanarSegment::StdVector::iterator it_j;
  big_map_segments_->clear();
  big_data_segments_->clear();
  for (it_i = map_segments_->begin(); it_i != map_segments_->end() -1 ; it_i++)
  {
    for (it_j = it_i + 1; it_j < map_segments_->end(); it_j++)
    {
      if (it_i->area < it_j->area)
      {
        pp = *it_i;
        *it_i = *it_j;
        *it_j = pp;
      }
    }
  }
  big_map_segments_->insert(big_map_segments_->begin(), map_segments_->begin(), map_segments_->begin() + N);

  for (it_i = data_segments_->begin(); it_i != data_segments_->end()-1; it_i++)
  {
    for (it_j = it_i + 1; it_j < data_segments_->end(); it_j++)
    {
      if (it_i->area < it_j->area)
      {
        pp = *it_i;
        *it_i = *it_j;
        *it_j = pp;
      }
    }
  }
  big_data_segments_->insert(big_data_segments_->begin(), data_segments_->begin(), data_segments_->begin() + N);

  for (it_i = big_map_segments_->begin(); it_i != big_map_segments_->end(); it_i++)
  {
    std::cout << it_i->area << " ";
  }
  std::cout << std::endl;
  for (it_i = big_data_segments_->begin(); it_i != big_data_segments_->end(); it_i++)
  {
    std::cout << it_i->area << " ";
  }
  std::cout << std::endl;
}

bool
Registration::findRotation(Solution solution, double value)
{
  Matrix3d rotation = Matrix3d::Zero();
  AreaConsistentPair::StdVector::iterator it;
  Matrix3d S = Matrix3d::Zero();
  //double total_area = solution.total_area;
  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;
  double weight = 0;
  for (it = solution.correspondences.begin(); it != solution.correspondences.end(); it++)
  {
    map_it = map_begin + it->lhs;
    data_it = data_begin + it->rhs;

    //weight = map_it->area < data_it->area ? map_it->area : data_it->area;
    weight = (map_it->area + data_it->area);
    //weight = 1.0;

    S += data_it->normal * map_it->normal.transpose() * weight;
  }

  JacobiSVD<Matrix3d> svd(S, ComputeFullU | ComputeFullV);
  rotation = svd.matrixV() * svd.matrixU().transpose();

  if (DEBUG)
  {
    std::cout << "difference between initial and refined rotation: \n" << rotation - solution.rotation << std::endl;
  }

  double norm1distance = (rotation - solution.rotation).cwiseAbs().sum();
  if (norm1distance > value)
  {
    return false;
  }

  solution.rotation = rotation;
  return true;
}

bool
Registration::findTranslation(Solution solution, double value)
{
  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;
  AreaConsistentPair::StdVector::iterator it;
  Vector3d translation = Vector3d::Zero();
  MatrixXd A(solution.correspondences.size(),3);
  VectorXd b(solution.correspondences.size());
  int i = 0;
  for (it = solution.correspondences.begin(); it != solution.correspondences.end(); it++)
  {
    A.row(i) = (map_begin + it->lhs)->normal.transpose();
    b(i) = (map_begin + it->lhs)->bias - (data_begin + it->rhs)->bias;
    //std::cout << A.row(i).dot(solution.translation) - b(i) << " ";
    i++;
  }
  translation = A.colPivHouseholderQr().solve(b);
  if (DEBUG)
  {
    std::cout << "difference between initial refined translation:\n" << (solution.translation - translation).transpose() << std::endl;
  }

  // if the difference between the initial translation and the translation calculated by all correspondences
  double norm1distance = (translation - solution.translation).cwiseAbs().sum();
  if (norm1distance > value)
  {
    return false;
  }

  solution.translation = translation;
  return true;
}

void
Registration::execute()
{
  DEBUG = false;
  boost::timer time;
  solutions_.clear();
  area_consistent_pair_triplets_.clear ();
  getBigSegments(params_.min_area);

  //filterByLinearity (50.0);

  if (params_.merge_angle != 0.0 || params_.merge_dis != 0.0)
  {
    mergeSurfacesOnSameInfinitePlane(cos(params_.merge_angle), params_.merge_dis);
  }

  //find all area-consistent planar segment pairs, in this setp, one segment can be consistent with multiple segments in another point cloud
  time.restart ();
  findAreaConsistentPlanes(params_.max_area_diff);
  std::cerr << "time elapsed: " << time.elapsed () << std::endl;

  //find all possible 3 non-parallel pairs, where each one contains three non-parallel planar segments.
  time.restart ();
  findAreaConsistentPairTriplets (0.0, params_.unparallel_min_angle, params_.unparallel_max_angle);
  std::cerr << "time elapsed for all possible three-non-planar pairs: " << time.elapsed () << std::endl;

  //find all potential solutions.
  time.restart ();
  findPotentialSolutions();
  std::cerr << "time elapsed for finding all potential solutions: " << time.elapsed () << std::endl;

  //find the solution which maxmize the spherical correlation
  time.restart ();
  findOptimumSolution ();
  //refineSolutions ();

  point2plane (solutions_[0]);
  rotation_ = solutions_[0].rotation;
  translation_ = solutions_[0].translation;

  std::cerr << "time elapsed for refine solutions: " << time.elapsed () << std::endl;
  std::cout << "Number of correspondences: " << solutions_[0].correspondences.size() << std::endl;

//  std::cout << "Candidate solutions number: " << solutions_.size() << std::endl;
//  std::cout << "rotation and translation: \n";
//  std::cout << solutions_[0].rotation << std::endl << solutions_[0].translation.transpose() << std::endl;
  transformPointcloud(data_cloud_, transformed_data_cloud_, solutions_[0].rotation, solutions_[0].translation);
}

void
Registration::findPotentialSolutions()
{
  if (solutions_.empty ())
    std::cerr << "No possible solution found, possibly there is something wrong with the thresholds\n";

  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();

  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;

  for (map_it = map_begin; map_it != big_map_segments_->end (); map_it++)
  {
    if (map_it->bias > 0)
    {
      continue;
    }
    map_it->bias = -map_it->bias;
    map_it->normal = -map_it->normal;
  }

  std::vector<Solution> solutions;

  for (size_t i = 0; i < solutions_.size (); i++)
  {
    Solution solution = solutions_[i];

    for (AreaConsistentPair::StdVector::iterator it = area_consistent_planes_->begin (); it != area_consistent_planes_->end (); it++)
    {
      map_it = map_begin + it->lhs;
      data_it = data_begin + it->rhs;

      /** consistent test using plane parameters and transformation. */
      if ((solution.rotation * data_it->normal).dot(map_it->normal) < cos(params_.max_angle_diff))
        continue;
      if (fabs(map_it->normal.dot(solution.translation) + data_it->bias - map_it->bias) > params_.max_bias_diff)
        continue;

      /** a simple overlapping test using the area and transformation. */
      if (overlapping (*it, solution.rotation, solution.translation) == false)
        continue;

      /** the area-consistent pair is accepted if it past all tests. */
      solution.correspondences.push_back(*it);
    }

    for (AreaConsistentPair::StdVector::iterator it = area_consistent_planes_->begin (); it != area_consistent_planes_->end (); it++)
    {
      map_it = map_begin + it->lhs;
      data_it = data_begin + it->rhs;

      /** If the two scan spots are on different side of one planar surface,
        * the normal of one segment shoule be changed to the opposite direction.
        * As a result, the two normal directions are aligned together.
        * This test can be performed because for Hessian plane (n,d) = (-n,-d).
        */
      if (-(solution.rotation * data_it->normal).dot(map_it->normal) < cos(params_.max_angle_diff))
        continue;
      if (fabs(map_it->normal.dot(solution.translation) - data_it->bias - map_it->bias) > params_.max_bias_diff)
        continue;

      /** a simple overlapping test using the area and transformation. */
      data_it->normal = -data_it->normal;
      data_it->bias = -data_it->bias;
      if (overlapping (*it, solution.rotation, solution.translation) == false)
        continue;

      /** the area-consistent pair is accepted if it past all tests. */
      solution.correspondences.push_back(*it);
    }

    ensureUniqueMapping(solution);

    if (solution.correspondences.size () >= 3)
    {
      if (findRotation (solution, 0.1) && findTranslation (solution, 0.1))
      {
        solutions.push_back (solution);
      }
    }
  }

  if (solutions.empty ())
  {
    std::cerr << "something error happened, no concensus set with more than 3 non-parallel segments found!\n";
    return;
  }

  std::cerr << solutions.size () << " potential solutions have been found.\n";
  solutions_.clear ();
  solutions_ = solutions;
}




void
Registration::findAreaConsistentPairTriplets(double simple_translation_test_threshold, double min_angle, double max_angle)
{
  /** Parallel and anti-parallel segments won't be used, min_angle and max_angle are two pre-set thresholds
      which are utilized to avoid nearly parallel or anti-parallel planar segments, usually we use 30 and 150 degrees. */
  double cos_min_angle = cos(min_angle);
  double cos_max_angle = cos(max_angle);

  double lhs_cos_angle = 0.0;
  double lhs_angle = 0.0;
  double rhs_cos_angle = 0.0;
  double rhs_angle = 0.0;
  double angle_dif = 0.0;

  Matrix3d rotation = Matrix3d::Zero();
  Vector3d translation = Vector3d::Zero();
  /** iterators which are used for planar segments access. */
  AreaConsistentPair::StdVector::iterator it1;
  AreaConsistentPair::StdVector::iterator it2;
  AreaConsistentPair::StdVector::iterator it3;

  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();

  PlanarSegment::StdVector::iterator map_it1;
  PlanarSegment::StdVector::iterator data_it1;
  PlanarSegment::StdVector::iterator map_it2;
  PlanarSegment::StdVector::iterator data_it2;
  PlanarSegment::StdVector::iterator map_it3;
  PlanarSegment::StdVector::iterator data_it3;

  Solution solution;
  //int pair1_overlapping = 0, pair2_overlapping = 0, pair3_overlapping = 0;
  /** First out loop, pick up an arbitrary area cosistent pair. */
  for (it1 = area_consistent_planes_->begin(); it1 != area_consistent_planes_->end()-1; it1++)
  {
    /** Secont out loop, pick up another area consistent pair. */
    for (it2 = it1 + 1; it2 != area_consistent_planes_->end(); it2++)
    {
      solution.correspondences.clear();
      solution.translation = Vector3d::Zero();
      solution.rotation = Matrix3d::Identity();

      // there should be no duplate planar segment.
      if (it1->lhs == it2->lhs || it1->rhs == it2->rhs)
        continue;

      map_it1 = map_begin + it1->lhs;
      map_it2 = map_begin + it2->lhs;
      data_it1 = data_begin + it1->rhs;
      data_it2 = data_begin + it2->rhs;

      //simple translation agreement test
      double translation_test1 =
          fabs(map_it1->bias - data_it1->bias) - fabs(map_it1->normal.dot(map_it2->normal)) * fabs(map_it2->bias - data_it2->bias);
      double translation_test2 =
          fabs(map_it2->bias - data_it2->bias) - fabs(map_it1->normal.dot(map_it2->normal)) * fabs(map_it1->bias - data_it1->bias);
      if (translation_test1 < simple_translation_test_threshold || translation_test2 < simple_translation_test_threshold)
        continue;

      //the two planes should not be parallel or anti-parallel
      lhs_cos_angle = map_it1->normal.dot(map_it2->normal);
      rhs_cos_angle = data_it1->normal.dot(data_it2->normal);
      if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
        continue;

      //the angle between two planes should remain similarly in two point clouds
      angle_dif = fabs(acos(lhs_cos_angle)-acos(rhs_cos_angle));
      if (angle_dif > params_.max_angle_diff)
        continue;

      rotation = rotationFromTwoCorrespondencePairs(it1,it2);
      if (exceedLocomotionAbility(rotation))
        continue;
      solution.correspondences.push_back(*it1);
      solution.correspondences.push_back(*it2);
      solution.rotation = rotation;

      //inner loop
      for (it3 = area_consistent_planes_->begin(); it3 != area_consistent_planes_->end(); it3++)
      {
        // there should be no duplicate area-consistent-pair
        if (it3 == it1 || it3 == it2)
          continue;
        //there should be no duplicate planar segment
        if (it3->lhs == it1->lhs || it3->rhs == it1->rhs)
          continue;
        if (it3->lhs == it2->lhs || it3->rhs == it2->rhs)
          continue;

        map_it3 = map_begin + it3->lhs;
        data_it3 = data_begin + it3->rhs;
        if ((rotation * data_it3->normal).dot(map_it3->normal) < cos(params_.max_angle_diff))
          continue;
        //planes should not be parallel or antpii-parallel to the first two planes
        lhs_cos_angle = map_it3->normal.dot(map_it1->normal);
        rhs_cos_angle = data_it3->normal.dot(data_it1->normal);
        if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
          continue;
        lhs_cos_angle = map_it3->normal.dot(map_it2->normal);
        rhs_cos_angle = data_it3->normal.dot(data_it2->normal);
        if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
          continue;

        /** The three normal vectors should not be on an infinite plane.
          * A vector parallel to the joint line of plane 1 and plane 2 can be computed as joint_line = normal_1.cross(normal_2).
          * The nomals are not on the same plane if this joint line is not parallel to plane 3.
          * In order to perform this test, the joint line is normalized and fixed to the origin, then it is projected to plane 3.
          * Afterwards, the angle between it and its projection on plane 3 is computed.
          */
        Vector3d joint_line = map_it1->normal.cross(map_it2->normal);
        joint_line = joint_line.normalized ();
        double dis_origin2plane = -map_it3->bias;
        double dis_endpoint2plane = joint_line.dot(map_it3->normal) - map_it3->bias;
        Vector3d projection_origin2plane = -dis_origin2plane * map_it3->normal;
        Vector3d projection_endpoint2plane = joint_line - dis_endpoint2plane * map_it3->normal;
        Vector3d projection = projection_endpoint2plane - projection_origin2plane;
        double cos_angle = projection.norm ();
        if (cos_angle > cos_min_angle || cos_angle < cos_max_angle)
          continue;

        translation = translationFromThreeCorrespondencePairs(it1, it2, it3);
        if (exceedLocomotionAbility(translation))
          continue;

        if (findRotation (solution,0.1) == false)
          continue;

        if (overlapping(*it1, solution.rotation, translation) == false)
          continue;
        //std::cerr << "pair 1 overlapping test: " << pair1_overlapping++ << std::endl;
        if (overlapping(*it2, solution.rotation, translation) == false)
          continue;
        //std::cerr << "pair 2 overlapping test: " << pair2_overlapping++ << std::endl;
        if (overlapping(*it3, solution.rotation, translation) == false)
          continue;
        //std::cerr << "pair 3 overlapping test: " << pair3_overlapping++ << std::endl;

        //sort it1,it2 and it3 in order to reach a unique combination.
        AreaConsistentPairTriplets key;
        if (it3 > it2)
        {
          key.first = it1 - area_consistent_planes_->begin();
          key.second = it2 - area_consistent_planes_->begin();
          key.third = it3 - area_consistent_planes_->begin();
        }
        if (it3 < it1)
        {
          key.first = it3 - area_consistent_planes_->begin();
          key.second = it1 - area_consistent_planes_->begin();
          key.third = it2 - area_consistent_planes_->begin();
        }
        if (it3 > it1 && it3 < it2)
        {
          key.first = it1 - area_consistent_planes_->begin();
          key.second = it3 - area_consistent_planes_->begin();
          key.third = it2 - area_consistent_planes_->begin();
        }

        if (area_consistent_pair_triplets_.find(key) == area_consistent_pair_triplets_.end ())
        {
          area_consistent_pair_triplets_.insert (key);
          solution.correspondences.push_back(*it3);
          solution.translation = translation;
          solutions_.push_back (solution);
        }
      }//inner loop
    }//second outer loop
  }//first outer loop

  std::cerr << solutions_.size () << " three-non-parallel pairs have been found.\n";
}

bool
Registration::overlapping (AreaConsistentPair one_pair,
                           Matrix3d rotation,
                           Vector3d translation)
{
  double area_l = (big_map_segments_->begin () + one_pair.lhs)->area;
  double area_r = (big_data_segments_->begin () + one_pair.rhs)->area;
  Vector3d mass_l = (big_map_segments_->begin () + one_pair.lhs)->mass_center;
  Vector3d mass_r = (big_data_segments_->begin () + one_pair.rhs)->mass_center;

  if ((rotation * mass_r + translation - mass_l).norm () > min(sqrt(area_l/M_PI), sqrt(area_r/M_PI)))
  {
    return false;
  }

  return true;
}

//bool
//Registration::overlapping(AreaConsistentPair::StdVector::iterator it1,
//                          AreaConsistentPair::StdVector::iterator it2,
//                          AreaConsistentPair::StdVector::iterator it3,
//                          Eigen::Matrix3d rotation,
//                          Eigen::Vector3d translation)
//{
//  double area_l_1 = (big_map_segments_->begin () + it1->lhs)->area;
//  double area_r_1 = (big_data_segments_->begin () + it1->rhs)->area;
//  Eigen::Vector3d mass_l_1 = (big_map_segments_->begin () + it1->lhs)->mass_center;
//  Eigen::Vector3d mass_r_1 = (big_data_segments_->begin () + it1->rhs)->mass_center;

//  if ((rotation * mass_r_1 + translation - mass_l_1).norm () > (sqrt(area_l_1/M_PI) + sqrt(area_r_1/M_PI)))
//  {
//    return false;
//  }

//  double area_l_2 = (big_map_segments_->begin () + it2->lhs)->area;
//  double area_r_2 = (big_data_segments_->begin () + it2->rhs)->area;
//  Eigen::Vector3d mass_l_2 = (big_map_segments_->begin () + it2->lhs)->mass_center;
//  Eigen::Vector3d mass_r_2 = (big_data_segments_->begin () + it2->rhs)->mass_center;

//  if ((rotation * mass_r_2 + translation - mass_l_2).norm () > (sqrt(area_l_2/M_PI) + sqrt(area_r_2/M_PI)))
//  {
//    return false;
//  }

//  double area_l_3 = (big_map_segments_->begin () + it3->lhs)->area;
//  double area_r_3 = (big_data_segments_->begin () + it3->rhs)->area;
//  Eigen::Vector3d mass_l_3 = (big_map_segments_->begin () + it3->lhs)->mass_center;
//  Eigen::Vector3d mass_r_3 = (big_data_segments_->begin () + it3->rhs)->mass_center;

//  if ((rotation * mass_r_3 + translation - mass_l_3).norm () > (sqrt(area_l_3/M_PI) + sqrt(area_r_3/M_PI)))
//  {
//    return false;
//  }

//  return true;
//}

//void
//Registration::mainloop(double simple_translation_test_threshold,
//                       double min_angle,
//                       double max_angle)
//{
//  //the transformation can only be computer with more than three non-parallel surfaces
//  int min_consensus_size = 3;
//  double cos_min_angle = cos(min_angle);
//  double cos_max_angle = cos(max_angle);

//  double lhs_cos_angle = 0.0;
//  double lhs_angle;
//  double rhs_cos_angle = 0.0;
//  double rhs_angle = 0.0;
//  double angle_dif = 0.0;
//  Matrix3d rotation = Matrix3d::Zero();
//  Vector3d translation = Vector3d::Zero();
//  AreaConsistentPair::StdVector::iterator it1;
//  AreaConsistentPair::StdVector::iterator it2;
//  AreaConsistentPair::StdVector::iterator it3;
//  AreaConsistentPair::StdVector::iterator it4;

//  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
//  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();

//  PlanarSegment::StdVector::iterator map_it1;
//  PlanarSegment::StdVector::iterator data_it1;
//  PlanarSegment::StdVector::iterator map_it2;
//  PlanarSegment::StdVector::iterator data_it2;
//  PlanarSegment::StdVector::iterator map_it3;
//  PlanarSegment::StdVector::iterator data_it3;
//  PlanarSegment::StdVector::iterator map_it4;
//  PlanarSegment::StdVector::iterator data_it4;

//  all_rotation_consistents_.clear();
//  all_translation_consistents_.clear();
//  Solution solution;
//  solutions_.clear();

//  /** First out loop, pick up an arbitrary area cosistent pair. */
//  for (it1 = area_consistent_planes_->begin(); it1 != area_consistent_planes_->end()-1; it1++)
//  {
//    /** Secont out loop, pick up another area consistent pair. */
//    for (it2 = it1 + 1; it2 != area_consistent_planes_->end(); it2++)
//    {
//      solution.correspondences.clear();
//      solution.translation = Vector3d::Zero();
//      solution.rotation = Matrix3d::Identity();

//      map_it1 = map_begin + it1->lhs;
//      map_it2 = map_begin + it2->lhs;
//      data_it1 = data_begin + it1->rhs;
//      data_it2 = data_begin + it2->rhs;

//      //simple translation agreement test
//      double translation_test1 =
//          fabs(map_it1->bias - data_it1->bias) - fabs(map_it1->normal.dot(map_it2->normal)) * fabs(map_it2->bias - data_it2->bias);
//      double translation_test2 =
//          fabs(map_it2->bias - data_it2->bias) - fabs(map_it1->normal.dot(map_it2->normal)) * fabs(map_it1->bias - data_it1->bias);
//      if (translation_test1 < simple_translation_test_threshold || translation_test2 < simple_translation_test_threshold)
//        continue;

//      //the two planes should not be parallel or anti-parallel
//      lhs_cos_angle = map_it1->normal.dot(map_it2->normal);
//      rhs_cos_angle = data_it1->normal.dot(data_it2->normal);
//      if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
//        continue;

//      //the angle between two planes should remain similarly in two point clouds
//      angle_dif = fabs(acos(lhs_cos_angle)-acos(rhs_cos_angle));
//      if (angle_dif > params_.max_angle_diff)
//        continue;

//      rotation = rotationFromTwoCorrespondencePairs(it1,it2);
//      if (exceedLocomotionAbility(rotation))
//        continue;
//      solution.correspondences.push_back(*it1);
//      solution.correspondences.push_back(*it2);
//      solution.rotation = rotation;

//      for (it3 = area_consistent_planes_->begin(); it3 != area_consistent_planes_->end(); it3++)
//      {

//        if (solution.correspondences.size()>2)
//        {
//            solution.correspondences.erase(solution.correspondences.begin()+2, solution.correspondences.end());
//        }

//        if (it3 == it1 || it3 == it2)
//          continue;
//        map_it3 = map_begin + it3->lhs;
//        data_it3 = data_begin + it3->rhs;
//        if ((rotation * data_it3->normal).dot(map_it3->normal) < cos(params_.max_angle_diff))
//          continue;
//        //planes should not be parallel or antpii-parallel to the first two planes
//        lhs_cos_angle = map_it3->normal.dot(map_it1->normal);
//        rhs_cos_angle = data_it3->normal.dot(data_it1->normal);
//        if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
//          continue;
//        lhs_cos_angle = map_it3->normal.dot(map_it2->normal);
//        rhs_cos_angle = data_it3->normal.dot(data_it2->normal);
//        if (lhs_cos_angle > cos_min_angle || lhs_cos_angle < cos_max_angle || rhs_cos_angle > cos_min_angle || rhs_cos_angle < cos_max_angle)
//          continue;

//        translation = translationFromThreeCorrespondencePairs(it1, it2, it3);
//        if (exceedLocomotionAbility(translation))
//          continue;

//        solution.correspondences.push_back(*it3);
//        solution.translation = translation;

//        for (it4 = area_consistent_planes_->begin(); it4 != area_consistent_planes_->end(); it4++)
//        {
//          if (it4 == it1 || it4 == it2 || it4 == it3)
//            continue;
//          map_it4 = map_begin + it4->lhs;
//          data_it4 = data_begin + it4->rhs;
//          if ((rotation * data_it4->normal).dot(map_it4->normal) < cos(params_.max_angle_diff))
//            continue;
//          if (fabs(map_it4->normal.dot(translation) + data_it4->bias - map_it4->bias) > params_.max_bias_diff)
//            continue;
//          solution.correspondences.push_back(*it4);
//        }

//        ensureUniqueMapping(solution);

//        if (solution.correspondences.size() >= min_consensus_size)
//        {
//          solutions_.push_back(solution);
//          min_consensus_size = solution.correspondences.size();
//        }
//      }
//    }
//  }
//  PCL_INFO ("%d solutions with consistent pairs (>3) have been found.\n", solutions_.size());

//  for (int i = 0; i < solutions_.size() - 1; i++)
//  {
//    for (int j = i + 1; j < solutions_.size(); j++)
//    {
//      if (solutions_[i].correspondences.size () < solutions_[j].correspondences.size())
//      {
//        solution = solutions_[i];
//        solutions_[i] = solutions_[j];
//        solutions_[j] = solution;
//      }
//    }
//  }

//  refineSolutions();
//  rotation_ = solutions_[0].rotation;
//  translation_ = solutions_[0].translation;
//}


void
Registration::ensureUniqueMapping(Solution &solution)
{
  bool DEBUG = false;
  if (DEBUG)
  {
    for (int i = 0; i < solution.correspondences.size(); i++)
      std::cout << "(" << solution.correspondences[i].lhs << "," << solution.correspondences[i].rhs << ");" ;
    std::cout << std::endl;
  }
  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
  std::vector<int> idx2remove;
  idx2remove.clear();
  std::vector<bool> key_visited;
  key_visited.clear ();
  key_visited.resize(solution.correspondences.size(), false);

  for (int i = 0; i < solution.correspondences.size() - 1; i++)
  {
    if (key_visited[i] || (solution.correspondences[i].lhs == -1))
    {
      continue;
    }
    key_visited[i] = true;

    int key_i = solution.correspondences[i].lhs;
    for (int j = i + 1; j < solution.correspondences.size(); j++)
    {
      int key_j = solution.correspondences[j].lhs;
      if (key_j != key_i)
        continue;
      key_visited[j] = true;
      Vector3d mass_lhs = (map_begin + key_i)->mass_center;
      Vector3d mass_rhs_i = (data_begin + solution.correspondences[i].rhs)->mass_center;
      Vector3d mass_rhs_j = (data_begin + solution.correspondences[j].rhs)->mass_center;
      double mass_dis_i = (solution.rotation * mass_rhs_i + solution.translation - mass_lhs).norm();
      double mass_dis_j = (solution.rotation * mass_rhs_j + solution.translation - mass_lhs).norm();
      if (mass_dis_i >= mass_dis_j)
      {
        solution.correspondences[i] = solution.correspondences[j];
      }
      solution.correspondences[j].lhs = solution.correspondences[j].rhs = -1;
      idx2remove.push_back(j);
    }
  }

  if (!idx2remove.empty())
  {
    AreaConsistentPair::StdVector correspondences;
    for (int k = 0; k < solution.correspondences.size(); k++)
    {
      if (solution.correspondences[k].lhs == -1)
        continue;
      correspondences.push_back(solution.correspondences[k]);
    }
    solution.correspondences.clear ();
    solution.correspondences = correspondences;
  }

  idx2remove.clear();
  key_visited.clear ();
  key_visited.resize(solution.correspondences.size(), false);

  for (int i = 0; i < solution.correspondences.size() - 1; i++)
  {
    if (key_visited[i] || (solution.correspondences[i].rhs == -1))
    {
      continue;
    }
    key_visited[i] = true;
    int key_i = solution.correspondences[i].rhs;
    for (int j = i + 1; j < solution.correspondences.size(); j++)
    {
      int key_j = solution.correspondences[j].rhs;
      if (key_j != key_i)
        continue;
      key_visited[j] = true;
      Vector3d mass_rhs = (data_begin + solution.correspondences[i].rhs)->mass_center;
      mass_rhs = solution.rotation * mass_rhs + solution.translation;
      Vector3d mass_lhs_i = (map_begin + solution.correspondences[i].lhs)->mass_center;
      Vector3d mass_lhs_j = (map_begin + solution.correspondences[j].lhs)->mass_center;
      double mass_dis_i = (mass_rhs - mass_lhs_i).norm();
      double mass_dis_j = (mass_rhs - mass_lhs_j).norm();
      if (mass_dis_i >= mass_dis_j)
      {
        solution.correspondences[i] = solution.correspondences[j];
      }
      solution.correspondences[j].lhs = solution.correspondences[j].rhs = -1;
      idx2remove.push_back(j);
    }
  }

  if (!idx2remove.empty())
  {
    AreaConsistentPair::StdVector correspondences;
    for (int k = 0; k < solution.correspondences.size(); k++)
    {
      if (solution.correspondences[k].lhs == -1)
        continue;
      correspondences.push_back(solution.correspondences[k]);
    }
    solution.correspondences.clear ();
    solution.correspondences = correspondences;
  }

  if (DEBUG)
  {
    for (int i = 0; i < solution.correspondences.size(); i++)
      std::cout << solution.correspondences[i].lhs << ", ";
    std::cout << std::endl;
    for (int i = 0; i < solution.correspondences.size(); i++)
      std::cout << solution.correspondences[i].rhs << ", ";
    std::cout << std::endl << std::endl;
  }
}

void
Registration::mergeSurfacesOnSameInfinitePlane(double min_dot_product,
                                               double max_bias_dif)
{
  PlanarSegment::StdVector::iterator it1,it2;
  PlanarSegment::StdVector segments;

  for (it1 = big_map_segments_->begin(); it1 != big_map_segments_->end()-1; it1++)
  {
    if (it1->area == 0.0)
      continue;
    for (it2 = it1+1; it2 != big_map_segments_->end(); it2++)
    {
      if (it2->area == 0.0)
        continue;
      if (fabs(it2->bias - it1->bias) > max_bias_dif)
        continue;
      if (it2->normal.dot(it1->normal) < min_dot_product)
        continue;

      /** \todo compute new plane parameter from scatter matrix. */
      it1->normal = it1->normal * it1->area / (it1->area + it2->area) + it2->normal * it2->area / (it1->area + it2->area);
      it1->normal = it1->normal.normalized();
      it1->bias = it1->bias * it1->area / (it1->area + it2->area) + it2->bias * it2->area / (it1->area + it2->area);
      it1->points.insert(it1->points.end(), it2->points.begin(), it2->points.end());
      it1->mass_center = (it1->mass_center * it1->point_num + it2->mass_center * it2->point_num) / (it1->point_num + it2->point_num);
      it1->area += it2->area;
      it2->area = 0.0;
      it2->normal = Vector3d::Zero();
    }
  }
  segments.clear();

  for (it1 = big_map_segments_->begin(); it1 != big_map_segments_->end(); it1++)
  {
    if (it1->area == 0.0)
      continue;
    segments.push_back(*it1);
  }
  big_map_segments_->clear();
  big_map_segments_->insert(big_map_segments_->begin(), segments.begin(), segments.end());

  for (it1 = big_data_segments_->begin(); it1 != big_data_segments_->end()-1; it1++)
  {
    if (it1->area == 0.0)
      continue;
    for (it2 = it1+1; it2 != big_data_segments_->end(); it2++)
    {
      if (it2->area == 0.0)
        continue;
      if (fabs(it2->bias - it1->bias) > max_bias_dif)
        continue;
      if (it2->normal.dot(it1->normal) < min_dot_product)
        continue;

      /** \todo compute new plane parameter from scatter matrix. */
      it1->normal = it1->normal * it1->area / (it1->area + it2->area) + it2->normal * it2->area / (it1->area + it2->area);
      it1->normal = it1->normal.normalized();
      it1->bias = it1->bias * it1->area / (it1->area + it2->area) + it2->bias * it2->area / (it1->area + it2->area);
      it1->area += it2->area;
      it1->points.insert(it1->points.end(), it2->points.begin(), it2->points.end());
      it2->area = 0.0;
      it2->normal = Vector3d::Zero();
    }
  }
  segments.clear();
  for (it1 = big_data_segments_->begin(); it1 != big_data_segments_->end(); it1++)
  {
    if (it1->area == 0)
      continue;
    segments.push_back(*it1);
  }
  big_data_segments_->clear();
  big_data_segments_->insert(big_data_segments_->begin(), segments.begin(), segments.end());

  PCL_INFO ("There are %d and %d segments in the map cloud and data cloud after merging planes, respectively.\n",
            big_map_segments_->size(), big_data_segments_->size());
}

bool
Registration::findOptimumSolution()
{
  Solution solution;

  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();

  AreaConsistentPair::StdVector::iterator it;
  double total_area = 0;

  /** sort the solutions by number of correspondences. */
  for (int i = 0; i < solutions_.size() - 1; i++)
  {
    for (int j = i + 1; j < solutions_.size(); j++)
    {
      if (solutions_[i].correspondences.size () < solutions_[j].correspondences.size())
      {
        solution = solutions_[i];
        solutions_[i] = solutions_[j];
        solutions_[j] = solution;
      }
    }
  }

  int index = 0;
  for (index = 0; index < solutions_.size() - 1; index++)
  {
    if (solutions_[index].correspondences.size() > solutions_[index+1].correspondences.size())
    {
      break;
    }
  }
  index += 1;

  if (solutions_.size () < 10)
  {
    index = solutions_.size ();
  }
  else // solutions_.size () > 10
  {
    if (index < 10)
      index = 10;
  }

  solutions_.erase (solutions_.begin () + index, solutions_.end ());

  findPotentialSolutions();

  for (int i = 0; i < index; i++)
  {
    solutions_[i].total_area = 0;
    total_area = 0;
    for (it = solutions_[i].correspondences.begin(); it != solutions_[i].correspondences.end(); it++)
    {
//      total_area += (map_begin + it->lhs)->area;
//      total_area += (data_begin + it->rhs)->area;
      total_area += (map_begin + it->lhs)->area * (data_begin + it->rhs)->area;
    }
    solutions_[i].total_area = total_area;
  }

  if (index > 1) // more than one solutions with same top number of correspondences
  {
    for (int i = 0; i < index - 1; i++)
    {
      for (int j = i + 1; j < index; j++)
      {
        if (solutions_[i].total_area < solutions_[j].total_area)
        {
          solution = solutions_[i];
          solutions_[i] = solutions_[j];
          solutions_[j] = solution;
        }
      }
    }
  }

  if (!findRotation (solutions_[0], 0.1))
  {
    std::cerr << "the difference between calculated rotation and initial rotation is bigger than the threshold.\n";
  }
  if (!findTranslation (solutions_[0], 0.1))
  {
    std::cerr << "the difference between calculated translation and initial translation is bigger than the threshold.\n";
  }
}

void
Registration::refineSolutions()
{
  Solution solution;
//  bool *map_flags = new bool [big_map_segments_->size()];
//  bool *data_flags = new bool [big_data_segments_->size()];

  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();

  AreaConsistentPair::StdVector::iterator it;
  double total_area = 0;

  int index = 0;
  for (int i = 0; i < solutions_.size() - 1 ; i++)
  {
    if (solutions_[i].correspondences.size() > solutions_[i+1].correspondences.size())
    {
      index = i + 1;
      break;
    }
  }

  PCL_INFO ("There are %d solutions with the top number of correspondences.\n", index);

  for (int i = 0; i < index; i++)
  {
    solutions_[i].total_area = 0;
    total_area = 0;
    for (it = solutions_[i].correspondences.begin(); it != solutions_[i].correspondences.end(); it++)
    {
//      total_area += (map_begin + it->lhs)->area;
//      total_area += (data_begin + it->rhs)->area;
      total_area += (map_begin + it->lhs)->area * (data_begin + it->rhs)->area;
    }
    solutions_[i].total_area = total_area;
  }

  if (index > 1) // more than one solutions with same top number of correspondences
  {
    for (int i = 0; i < index - 1; i++)
    {
      for (int j = i + 1; j < index; j++)
      {
        if (solutions_[i].total_area < solutions_[j].total_area)
        {
          solution = solutions_[i];
          solutions_[i] = solutions_[j];
          solutions_[j] = solution;
        }
      }
    }
  }

  if (!findRotation (solutions_[0], 0.1))
  {
    std::cerr << "the difference between calculated rotation and initial rotation is bigger than the threshold.\n";
  }
  if (!findTranslation (solutions_[0], 0.1))
  {
    std::cerr << "the difference between calculated translation and initial translation is bigger than the threshold.\n";
  }
}


void
Registration::point2plane(Solution solution)
{
  AreaConsistentPair::StdVector::iterator it;
  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;

  Matrix6d C = Matrix6d::Zero ();
  Vector6d b = Vector6d::Zero ();
  Vector3d g_ij, n_i;
  Vector6d gn_ij;

  for (it = solution.correspondences.begin(); it != solution.correspondences.end(); it++)
  {
    map_it = map_begin + it->lhs;
    data_it = data_begin + it->rhs;
    n_i = map_it->normal;
    for (std::vector<int>::iterator idx = data_it->points.begin(); idx != data_it->points.end(); idx++)
    {
      Vector3d point(data_cloud_->points[*idx].x, data_cloud_->points[*idx].y, data_cloud_->points[*idx].z);
      point = solution.rotation * point + solution.translation;
      g_ij = point.cross (n_i);
      gn_ij.head<3>() = g_ij;
      gn_ij.tail<3>() = n_i;
      C += gn_ij * gn_ij.transpose ();

      double dis = point.dot (n_i) - map_it->bias;
      b += dis * gn_ij;
    }
  }

  Vector6d r = C.fullPivHouseholderQr ().solve(b);
  assert(b.isApprox(C*r));

  Matrix3d rotation;
  double cosx = cos(r(0)), sinx = sin(r(0));
  double cosy = cos(r(1)), siny = sin(r(1));
  double cosz = cos(r(2)), sinz = sin(r(2));
  rotation << cosy*cosz, cosx*sinz + sinx*siny*cosz, sinx*sinz - cosx*siny*cosz,
              -cosy*sinz, cosx*cosz - sinx*siny*sinz, sinx*cosz + cosx*siny*sinz,
              siny, -sinx*cosy, cosx*cosy;

  solution.rotation = rotation * solution.rotation;
  solution.translation = rotation * solution.translation + r.tail<3>();

//  std::cout << "eular angles and translation: " << r.transpose () << std::endl;
}


Vector3d
Registration::translationFromThreeCorrespondencePairs(AreaConsistentPair::StdVector::iterator it1,
                                                      AreaConsistentPair::StdVector::iterator it2,
                                                      AreaConsistentPair::StdVector::iterator it3)
{
  Vector3d translation = Vector3d::Zero();
  Matrix3d A = Matrix3d::Zero();
  Vector3d b = Vector3d::Zero();

  A.row(0) = (big_map_segments_->begin() + it1->lhs)->normal.transpose();
  A.row(1) = (big_map_segments_->begin() + it2->lhs)->normal.transpose();
  A.row(2) = (big_map_segments_->begin() + it3->lhs)->normal.transpose();

  b(0) = (big_map_segments_->begin() + it1->lhs)->bias - (big_data_segments_->begin() + it1->rhs)->bias;
  b(1) = (big_map_segments_->begin() + it2->lhs)->bias - (big_data_segments_->begin() + it2->rhs)->bias;
  b(2) = (big_map_segments_->begin() + it3->lhs)->bias - (big_data_segments_->begin() + it3->rhs)->bias;

//  std::cout << (big_map_segments_->begin() + it1->lhs)->normal.transpose() << std::endl;
//  std::cout << (big_map_segments_->begin() + it2->lhs)->normal.transpose() << std::endl;
//  std::cout << (big_map_segments_->begin() + it3->lhs)->normal.transpose() << std::endl;
//
//  std::cout << A << std::endl;
  translation = A.colPivHouseholderQr().solve(b);

//  std::cout << "A:\n" << A.row(0) << std::endl << A.row(1) << std::endl << A.row(2) << std::endl
//      << "b:" << b(0) << " " << b(1) << " " << b(2) << std::endl
//      << "translation:" << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;

//  std::cout << "(" << (A*translation)(0) << "," << (A*translation)(1) << "," << (A*translation)(2) << ")"
//      << ", (" << b(0) << "," << b(1) << "," << b(2) << ")\n"
  return translation;
}

Vector3d
Registration::translationFromThreeCorrespondencePairs(AreaConsistentPair pair1,
                                                      AreaConsistentPair pair2,
                                                      AreaConsistentPair pair3)
{
  Vector3d translation = Vector3d::Zero();
  Matrix3d A = Matrix3d::Zero();
  Vector3d b = Vector3d::Zero();

  A.row(0) = (big_map_segments_->begin() + pair1.lhs)->normal.transpose();
  A.row(1) = (big_map_segments_->begin() + pair2.lhs)->normal.transpose();
  A.row(2) = (big_map_segments_->begin() + pair3.lhs)->normal.transpose();

  b(0) = (big_map_segments_->begin() + pair1.lhs)->bias - (big_data_segments_->begin() + pair1.rhs)->bias;
  b(1) = (big_map_segments_->begin() + pair2.lhs)->bias - (big_data_segments_->begin() + pair2.rhs)->bias;
  b(2) = (big_map_segments_->begin() + pair3.rhs)->bias - (big_data_segments_->begin() + pair3.rhs)->bias;

  translation = A.colPivHouseholderQr().solve(b);

//  std::cout << "A:\n" << A.row(0) << std::endl << A.row(1) << std::endl << A.row(2) << std::endl
//      << "b:" << b(0) << " " << b(1) << " " << b(2) << std::endl
//      << "translation:" << translation(0) << " " << translation(1) << " " << translation(2) << std::endl;
//  std::cout << "(" << (A*translation)(0) << "," << (A*translation)(1) << "," << (A*translation)(2) << ")"
//      << ", (" << b(0) << "," << b(1) << "," << b(2) << ")\n";
  if (translation.norm() > params_.max_translation_norm)
    translation = Vector3d::Zero();

  return translation;
}


Matrix3d
Registration::rotationFromTwoCorrespondencePairs(AreaConsistentPair pair1,
                                                 AreaConsistentPair pair2)
{
  Vector3d axis_1 = Vector3d::Zero();
  Vector3d axis_2 = Vector3d::Zero();
  double angle_1 = 0.0;
  double angle_2 = 0.0;
  AngleAxis<double > rotation_1;
  AngleAxis<double > rotation_2;
  Matrix3d rotation_matrix_1 = Matrix3d::Zero();
  Matrix3d rotation_matrix_2 = Matrix3d::Zero();
  Matrix3d rotation_matrix = Matrix3d::Zero();
  Vector3d lhs_normal_1 = Vector3d::Zero();
  Vector3d lhs_normal_2 = Vector3d::Zero();
  Vector3d rhs_normal_1 = Vector3d::Zero();
  Vector3d rhs_normal_2 = Vector3d::Zero();

  Vector3d lhs_intersection_line = Vector3d::Zero();
  Vector3d rhs_intersection_line = Vector3d::Zero();

  lhs_normal_1 = (big_map_segments_->begin() +  pair1.lhs)->normal;
  lhs_normal_2 = (big_map_segments_->begin() +  pair2.lhs)->normal;
  rhs_normal_1 = (big_data_segments_->begin() + pair1.rhs)->normal;
  rhs_normal_2 = (big_data_segments_->begin() + pair2.rhs)->normal;

  lhs_intersection_line = lhs_normal_1.cross(lhs_normal_2);
  lhs_intersection_line = lhs_intersection_line.normalized();
  rhs_intersection_line = rhs_normal_1.cross(rhs_normal_2);
  rhs_intersection_line = rhs_intersection_line.normalized();

  axis_1 = rhs_intersection_line.cross(lhs_intersection_line);
  axis_1 = axis_1.normalized();

  angle_1 = acos(rhs_intersection_line.dot(lhs_intersection_line));

  rotation_1 = AngleAxis<double >(angle_1,axis_1);
  rotation_matrix_1 = rotation_1.toRotationMatrix ();

  if ((rotation_matrix_1 * rhs_intersection_line).dot(lhs_intersection_line)<0.9999)
  {
    rotation_1 = AngleAxis<double >(-angle_1, axis_1);
    rotation_matrix_1 = rotation_1.toRotationMatrix ();
  }

//    std::cout << (rotation_matrix_1 * rhs_intersection_line).dot(lhs_intersection_line) << " " <<
//        acos((rotation_matrix_1 * rhs_normal_1).dot(lhs_normal_1)) * 180 / M_PI << " " <<
//        acos((rotation_matrix_1 * rhs_normal_2).dot(lhs_normal_2)) * 180 / M_PI << "\n";

  angle_2 = (acos((rotation_matrix_1 * rhs_normal_1).dot(lhs_normal_1)) +
      acos((rotation_matrix_1 * rhs_normal_2).dot(lhs_normal_2)))/2;
  axis_2 = lhs_intersection_line;

  rotation_2 = AngleAxis<double >(angle_2, axis_2);
  rotation_matrix_2 = rotation_2.toRotationMatrix ();
  if ((rotation_matrix_2 * rotation_matrix_1 * rhs_normal_1).dot(lhs_normal_1) < 0.9980 ||
      (rotation_matrix_2 * rotation_matrix_1 * rhs_normal_2).dot(lhs_normal_2) < 0.9980)
  {
    rotation_2 = AngleAxis<double >(-angle_2, axis_2);
    rotation_matrix_2 = rotation_2.toRotationMatrix ();
  }
  rotation_matrix = rotation_2 * rotation_1;
  return rotation_matrix;
}


Matrix3d
Registration::rotationFromTwoCorrespondencePairs(AreaConsistentPair::StdVector::iterator it1,
                                                AreaConsistentPair::StdVector::iterator it2)
{
  Vector3d axis_1 = Vector3d::Zero();
  Vector3d axis_2 = Vector3d::Zero();
  double angle_1 = 0.0;
  double angle_2 = 0.0;
  double angle_2_1 = 0.0;
  double angle_2_2 = 0.0;
  double lhs_area_1, lhs_area_2, rhs_area_1, rhs_area_2;
  AngleAxis<double > rotation_1;
  AngleAxis<double > rotation_2;
  Matrix3d rotation_matrix_1 = Matrix3d::Identity();
  Matrix3d rotation_matrix_2 = Matrix3d::Identity();
  Matrix3d rotation_matrix = Matrix3d::Identity();
  Vector3d lhs_normal_1 = Vector3d::Zero();
  Vector3d lhs_normal_2 = Vector3d::Zero();
  Vector3d rhs_normal_1 = Vector3d::Zero();
  Vector3d rhs_normal_2 = Vector3d::Zero();

  Vector3d lhs_intersection_line = Vector3d::Zero();
  Vector3d rhs_intersection_line = Vector3d::Zero();

  lhs_normal_1 = (big_map_segments_->begin() +  it1->lhs)->normal;
  lhs_normal_2 = (big_map_segments_->begin() +  it2->lhs)->normal;
  rhs_normal_1 = (big_data_segments_->begin() + it1->rhs)->normal;
  rhs_normal_2 = (big_data_segments_->begin() + it2->rhs)->normal;
  lhs_area_1 = (big_map_segments_->begin() +  it1->lhs)->area;
  lhs_area_2 = (big_map_segments_->begin() +  it2->lhs)->area;
  rhs_area_1 = (big_data_segments_->begin() + it1->rhs)->area;
  rhs_area_2 = (big_data_segments_->begin() + it2->rhs)->area;

  lhs_intersection_line = (lhs_normal_1.cross(lhs_normal_2)).normalized();
  rhs_intersection_line = (rhs_normal_1.cross(rhs_normal_2)).normalized();

  axis_1 = (rhs_intersection_line.cross(lhs_intersection_line)).normalized();
  angle_1 = acos(rhs_intersection_line.dot(lhs_intersection_line));

  rotation_1 = AngleAxis<double >(angle_1,axis_1);
  rotation_matrix_1 = rotation_1.toRotationMatrix ();

  if ((rotation_matrix_1 * rhs_intersection_line).dot(lhs_intersection_line)<0.9999)
  {
    rotation_1 = AngleAxis<double >(-angle_1, axis_1);
    rotation_matrix_1 = rotation_1.toRotationMatrix ();
  }

  //std::cout << (rotation_matrix_1 * rhs_intersection_line).dot(lhs_intersection_line) << " ";
  /** \todo how to compute a rotation which can align the two normals. */
  angle_2_1 = acos((rotation_matrix_1 * rhs_normal_1).dot(lhs_normal_1));
  angle_2_2 = acos((rotation_matrix_1 * rhs_normal_2).dot(lhs_normal_2));
  angle_2 = ((lhs_area_1 + rhs_area_1) * angle_2_1 + (lhs_area_2 + rhs_area_2) * angle_2_2) / (lhs_area_1 + lhs_area_2 + rhs_area_1 + rhs_area_2);
  axis_2 = lhs_intersection_line;

  rotation_2 = AngleAxis<double >(angle_2, axis_2);
  rotation_matrix_2 = rotation_2.toRotationMatrix ();
  if ((rotation_matrix_2 * rotation_matrix_1 * rhs_normal_1).dot(lhs_normal_1) < 0.9980 ||
      (rotation_matrix_2 * rotation_matrix_1 * rhs_normal_2).dot(lhs_normal_2) < 0.9980)
  {
    rotation_2 = AngleAxis<double >(-angle_2, axis_2);
    rotation_matrix_2 = rotation_2.toRotationMatrix ();
  }
  rotation_matrix = rotation_2 * rotation_1;
  return rotation_matrix;
}

void
Registration::setMapSegments(PlanarSegment::StdVectorPtr segments)
{
  map_segments_ = segments;
}


void
Registration::setMapSegments(std::string segments_file)
{
  double tmp;
  std::ifstream infile;
  infile.open(segments_file.c_str());
  infile >> tmp >> tmp >> tmp;
  int pp_size = 0;
  infile >> pp_size;
  map_segments_->clear();
  map_segments_->resize(pp_size);
  int pp_index = 0;
  PlanarSegment::StdVector::iterator it = map_segments_->begin();
  while (!infile.eof ())
  {
    infile >> it->normal(0) >> it->normal(1) >> it->normal(2) >> it->bias >> it->mse >>
        it->area >> it->point_num >>it->mass_center(0) >> it->mass_center(1) >> it->mass_center(2);
    it ++;
  }
  infile.close ();
  PCL_INFO ("%d raw map segments loaded.\n", map_segments_->size());
}

void
Registration::setDataSegments(PlanarSegment::StdVectorPtr segments)
{
  data_segments_ = segments;
}


void
Registration::setDataSegments(std::string segments_file)
{
  double tmp;
  std::ifstream infile;
  infile.open(segments_file.c_str());
  infile >> tmp >> tmp >> tmp;
  int pp_size = 0;
  infile >> pp_size;
  data_segments_->clear();
  data_segments_->resize(pp_size);
  int pp_index = 0;
  PlanarSegment::StdVector::iterator it = data_segments_->begin();
  while (!infile.eof ())
  {
    infile >> it->normal(0) >> it->normal(1) >> it->normal(2) >> it->bias >> it->mse >>
        it->area >> it->point_num >>it->mass_center(0) >> it->mass_center(1) >> it->mass_center(2);
    it ++;
  }
  infile.close ();
  PCL_INFO ("%d raw data segments loaded.\n", data_segments_->size());
}


void
Registration::visualizeCorrespondences()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
  map_vis->resize(map_cloud_->size());
  data_vis->resize(transformed_data_cloud_->size());

  std::vector<RGB> colors;
  getColors(colors);
  AreaConsistentPair::StdVector::iterator it;
  PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
  PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
  PlanarSegment::StdVector::iterator map_it;
  PlanarSegment::StdVector::iterator data_it;
  srand ( time(NULL) );
  int cnt_map = 0, cnt_data = 0;
  for (it = solutions_[0].correspondences.begin(); it != solutions_[0].correspondences.end(); it++)
  {
      map_it = map_begin + it->lhs;
      data_it = data_begin + it->rhs;
      int gray = 255;
      int color_index;

      do{
        color_index = rand() % colors.size();
        gray = (colors[color_index].r + colors[color_index].g + colors[color_index].b) / 3;
      }
      while (gray < 50 || gray > 200);
      for (int j = 0; j < map_it->points.size(); j++)
      {
        int index = map_it->points[j];
        map_vis->points[cnt_map].x = map_cloud_->points[index].x;
        map_vis->points[cnt_map].y = map_cloud_->points[index].y;
        map_vis->points[cnt_map].z = map_cloud_->points[index].z;
        map_vis->points[cnt_map].r = colors[color_index].r;
        map_vis->points[cnt_map].g = colors[color_index].g;
        map_vis->points[cnt_map].b = colors[color_index].b;
        cnt_map ++;
      }
      for (int j = 0; j < data_it->points.size(); j++)
      {
        int index = data_it->points[j];
        data_vis->points[cnt_data].x = transformed_data_cloud_->points[index].x;
        data_vis->points[cnt_data].y = transformed_data_cloud_->points[index].y;
        data_vis->points[cnt_data].z = transformed_data_cloud_->points[index].z;
        data_vis->points[cnt_data].r = colors[color_index].r;
        data_vis->points[cnt_data].g = colors[color_index].g;
        data_vis->points[cnt_data].b = colors[color_index].b;
        cnt_data ++;
      }
  }

  map_vis_->points.erase(map_vis_->points.begin()+cnt_map, map_vis_->points.end());
  map_vis_->width = cnt_map;
  map_vis_->height = 1;
  map_vis_->resize(cnt_map);

  data_vis_->points.erase(data_vis_->points.begin()+cnt_data, data_vis_->points.end());
  data_vis_->width = cnt_data;
  data_vis_->height = 1;
  data_vis_->resize(cnt_data);

  pcl::visualization::PCLVisualizer viewer ("visualization for translation");
  int vp[2] = {1,2};
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, vp[0]);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, vp[1]);
  viewer.setBackgroundColor (1,1,1);

  viewer.addPointCloud (map_vis, "map_vis", vp[0]);
  viewer.addPointCloud (data_vis, "data_vis", vp[0]);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_vis");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "data_vis");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "map_vis");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "data_vis");


  for (it = solutions_[0].correspondences.begin(); it != solutions_[0].correspondences.end(); it++)
  {
      map_it = map_begin + it->lhs;
      data_it = data_begin + it->rhs;
      std::ostringstream text;
      pcl::PointXYZ position;
      text << "m" << it - solutions_[0].correspondences.begin();
      position.x = map_it->mass_center (0);
      position.y = map_it->mass_center (1);
      position.z = map_it->mass_center (2);
      viewer.addText3D (text.str (), position, 0.4, 0.0, 0.0, 0.0, text.str (), 0);
      std::ostringstream text1;
      text1 << "d" << it - solutions_[0].correspondences.begin();
      Vector3d mass_center = solutions_[0].rotation * data_it->mass_center + solutions_[0].translation;
      position.x = mass_center (0);
      position.y = mass_center (1);
      position.z = mass_center (2);
      viewer.addText3D (text1.str (), position, 0.4, 0.0, 0.0, 0.0, text1.str (), 0);
  }

  viewer.addPointCloud (map_cloud_, "map_cloud", vp[1]);
  viewer.addPointCloud (transformed_data_cloud_, "transformed_data_cloud", vp[1]);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_data_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "map_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "transformed_data_cloud");

  viewer.spin ();
}


void
Registration::visualizeCorrespondencesWithPoints()
{  pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr data_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
   map_vis->resize(map_cloud_->size());
   data_vis->resize(transformed_data_cloud_->size());

   std::vector<RGB> colors;
   getColors(colors);
   AreaConsistentPair::StdVector::iterator it;
   PlanarSegment::StdVector::iterator map_begin = big_map_segments_->begin();
   PlanarSegment::StdVector::iterator data_begin = big_data_segments_->begin();
   PlanarSegment::StdVector::iterator map_it;
   PlanarSegment::StdVector::iterator data_it;
   srand ( time(NULL) );
   int cnt_map = 0, cnt_data = 0;
   for (it = solutions_[0].correspondences.begin(); it != solutions_[0].correspondences.end(); it++)
   {
       map_it = map_begin + it->lhs;
       data_it = data_begin + it->rhs;
       int gray = 255;
       int color_index;

       do{
         color_index = rand() % colors.size();
         gray = (colors[color_index].r + colors[color_index].g + colors[color_index].b) / 3;
       }
       while (gray < 100 || gray > 200);
       for (int j = 0; j < map_it->points.size(); j++)
       {
         int index = map_it->points[j];
         map_vis->points[cnt_map].x = map_cloud_->points[index].x;
         map_vis->points[cnt_map].y = map_cloud_->points[index].y;
         map_vis->points[cnt_map].z = map_cloud_->points[index].z;
         map_vis->points[cnt_map].r = colors[color_index].r;
         map_vis->points[cnt_map].g = colors[color_index].g;
         map_vis->points[cnt_map].b = colors[color_index].b;
         cnt_map ++;
       }
       for (int j = 0; j < data_it->points.size(); j++)
       {
         int index = data_it->points[j];
         data_vis->points[cnt_data].x = transformed_data_cloud_->points[index].x;
         data_vis->points[cnt_data].y = transformed_data_cloud_->points[index].y;
         data_vis->points[cnt_data].z = transformed_data_cloud_->points[index].z;
         data_vis->points[cnt_data].r = colors[color_index].r;
         data_vis->points[cnt_data].g = colors[color_index].g;
         data_vis->points[cnt_data].b = colors[color_index].b;
         cnt_data ++;
       }
   }

   map_vis_->points.erase(map_vis_->points.begin()+cnt_map, map_vis_->points.end());
   map_vis_->width = cnt_map;
   map_vis_->height = 1;
   map_vis_->resize(cnt_map);

   data_vis_->points.erase(data_vis_->points.begin()+cnt_data, data_vis_->points.end());
   data_vis_->width = cnt_data;
   data_vis_->height = 1;
   data_vis_->resize(cnt_data);

   pcl::visualization::PCLVisualizer viewer ("visualization for translation");
   viewer.setBackgroundColor (1,1,1);

   viewer.addPointCloud (map_cloud_, "map_cloud", 0);
   viewer.addPointCloud (transformed_data_cloud_, "transformed_data_cloud", 0);
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "map_cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_data_cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "map_cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "transformed_data_cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "map_cloud");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "transformed_data_cloud");

   viewer.addPointCloud (map_vis, "map_vis", 0);
   viewer.addPointCloud (data_vis, "data_vis", 0);
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map_vis");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "data_vis");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "map_vis");
   viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "data_vis");

   viewer.spin ();
}


void
Registration::visualization( )
{
  pcl::visualization::PCLVisualizer viewer ("visualization for translation");
  viewer.addPointCloud (map_cloud_, "map_cloud", 0);
  viewer.addPointCloud (transformed_data_cloud_, "transformed_data_cloud", 0);
  viewer.addCoordinateSystem (1.0);
  viewer.setBackgroundColor (1,1,1);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_data_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "map_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "transformed_data_cloud");
  viewer.spin ();
}

void
Registration::rotatePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                               Matrix3d rotation)
{
  double rm[9];
  rm[0] = rotation(0,0);
  rm[1] = rotation(0,1);
  rm[2] = rotation(0,2);
  rm[3] = rotation(1,0);
  rm[4] = rotation(1,1);
  rm[5] = rotation(1,2);
  rm[6] = rotation(2,0);
  rm[7] = rotation(2,1);
  rm[8] = rotation(2,2);
  output->resize(input->size());
  for (size_t i = 0; i < input->size(); i++)
  {
    output->points[i].x = rm[0] * input->points[i].x + rm[1] * input->points[i].y + rm[2] * input->points[i].z;
    output->points[i].y = rm[3] * input->points[i].x + rm[4] * input->points[i].y + rm[5] * input->points[i].z;
    output->points[i].z = rm[6] * input->points[i].x + rm[7] * input->points[i].y + rm[8] * input->points[i].z;
  }
}


void
Registration::transformPointcloud(PointCloudPtr input,
                                  PointCloudPtr output,
                                  Matrix3d rotation,
                                  Vector3d trans)
{
  double rm[9];
  double translation[3];
  rm[0] = rotation(0,0);
  rm[1] = rotation(0,1);
  rm[2] = rotation(0,2);
  rm[3] = rotation(1,0);
  rm[4] = rotation(1,1);
  rm[5] = rotation(1,2);
  rm[6] = rotation(2,0);
  rm[7] = rotation(2,1);
  rm[8] = rotation(2,2);
  translation[0] = trans(0);
  translation[1] = trans(1);
  translation[2] = trans(2);
  output->resize(input->size());
  for (size_t i = 0; i < input->size(); i++)
  {
    output->points[i].x = rm[0] * input->points[i].x + rm[1] * input->points[i].y + rm[2] * input->points[i].z + translation[0];
    output->points[i].y = rm[3] * input->points[i].x + rm[4] * input->points[i].y + rm[5] * input->points[i].z + translation[1];
    output->points[i].z = rm[6] * input->points[i].x + rm[7] * input->points[i].y + rm[8] * input->points[i].z + translation[2];
  }
}
