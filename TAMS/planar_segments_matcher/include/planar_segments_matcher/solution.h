#ifndef SOLUTION_H_
#define SOLUTION_H_

#include <eigen3/Eigen/Core>

#include "planar_segments_matcher/correspondences.h"

namespace tams
{
  /** \brief This class represents the found solution by the spherical correlation maximizer class. */
  class Solution
  {
    public:
      Solution();
      ~Solution();

    private:
      Correspondences correspondences_;
      Eigen::Vector3d translation_;
      Eigen::Matrix3d rotation_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif // SOLUTION_H
