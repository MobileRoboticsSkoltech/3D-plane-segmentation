#ifndef ONE_PAIR_TESTER_H_
#define ONE_PAIR_TESTER_H_

#include <eigen3/Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "abstract_planar_segment/abstract_planar_segment.h"
#include "planar_segments_matcher/matcher_parameters.h"

namespace tams
{
  class OnePairTester
  {
    public:
      typedef boost::shared_ptr<OnePairTester> Ptr;

    public:
      /** \brief Empty constructor. */
      OnePairTester();

      /** \brief Constructor for two given segments.
          \param ps_1 is a boost shared pointer to one planar segment
          \param ps_2 is a boost shared pointer to another planar segment
      */
      OnePairTester(AbstractPlanarSegment::Ptr ps_1, AbstractPlanarSegment::Ptr ps_2);

      /** \brief Tests if the odometry rotation agrees within threshold to the one that can be inferred from ps1 and ps2.
       * \param threshold is the provided threshold.
       * \param odometry_2_1 transformation computed from odometry
       * \param value is the returned chi-square value.
       * \return if the test succeeded.
       */
      bool
      odometryRotationAgreement (double threshold,
                                 const Eigen::Matrix4d odometry_2_1,
                                 const Eigen::Matrix3d rotation_predicted,
                                 double &value);

      /** Tests if the odometry translation agrees within threshold to the one that can be
       * inferred from ps1 and ps2.
       * \param threshold is the provided threshold.
       * \param odometry_2_1 is the transformation computed from odometry readings.
       * \param value is the returned chi-square value.
       * \return if the test succeeded.
       */
      bool
      odometryTranslationAgreement (double threshold,
                                    const Eigen::Matrix4d odometry_2_1,
                                    double &value);

      /** \brief Test whether two segments have similar area.
       * \param treshold is the provided threshold.
       * \param value is the computed metric which was compared with threshold.
       * \return if the two planar patches are similar in area within threshold.
       */
      bool
      areaSimilarityAgreement (double threshold, double & value);

      /** \brief Test whether the two planar segments can be considered parallel or anti-parallel.
       *  \param threshold is the provided threshold.
       *  \param value is the returned metric.
       *  \return if the two patches can be considered parallel or anti-parallel based on a chi-square test.
       */

      bool
      areParallel (double threshold,
                   double &value);

      /** \brief Test whether the two planar segments are non-parallel.
       *  The relation between two segments are classified into three categories: parallel, non-parallel and uncertain.
       *  Two planar segments are considered to be non-parallel only if
       *  min_angle <= their intersection angle <= max_angle.
       *  \param min_angle the provided threshold in degree.
       *  \param max_angle the provided threshold in degree.
       *  \param value the computed intesection angle in degree between ps_1_ and ps_2_.
       *  \return true if non-parallel and false otherwise. */
      bool
      areNonparallel(double min_angle,
                     double max_angle,
                     double &value);

      /** \brief Test whether ps_1 and ps_2 are consistent with the given translation and its covariance.
       *  \param threshold is the given threshold.
       *  \param translation_2_1 is the given predicted translation
       *  \param value is the computed metric with regards to the chi-square test.
       *  \return if ps1 and ps2 are consistent with the given translation and its covariance.
       */
      bool
      consistentWithTranslation (double threshold,
                                 const Eigen::Vector3d translation_2_1,
                                 const Eigen::Matrix3d cov_translation_2_1,
                                 double &value);

      /** \brief Test whether ps_1_ and ps_2_ are consistent with the given rotation.
       * It is like the parallel-ness test for n1 and rotation_2_1(n2). It is just based on dot product and succeeds
       * if dot_product >= threshold.
       * \param threshold is the given threshold.
       * \param rotation_2_1 is the predicted rotation.
       * \param value is the n1.dot(rotation_2_1*n2).
       * \return true if consitent, false otherwise.
       */
      bool
      consistentWithRotation (double threshold,
                              const Eigen::Matrix3d rotation_2_1,
                              double &value);

    private:
      AbstractPlanarSegment::Ptr ps_1_;
      AbstractPlanarSegment::Ptr ps_2_;
  };
}


#endif // ONE_PAIR_TESTER_H
