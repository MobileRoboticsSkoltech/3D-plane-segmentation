#ifndef TWO_PAIRS_TESTER_H_
#define TWO_PAIRS_TESTER_H_

#include <eigen3/Eigen/Core>
#include <boost/shared_ptr.hpp>
#include "common/abstract_planar_segment.h"

namespace tams
{
  class TwoPairsTester
  {
    public:
      /** \brief Empty constructor. */
      TwoPairsTester();

      /** \brief Constructor. */
      TwoPairsTester(AbstractPlanarSegment::Ptr p1_1,
                     AbstractPlanarSegment::Ptr p1_2,
                     AbstractPlanarSegment::Ptr p2_1,
                     AbstractPlanarSegment::Ptr p2_2);

      /** \brief Empty deconstructor. */
      ~TwoPairsTester();

      /** Assuming that the rotation is given as Rfixed, finds if the
       * pairs overlap. Using _f for fixed, _p for potential,
       * p1_1 is pL_f, p1_2 is pL_p, p2_1 is pR_f, p2_2 is pR_p
       * \param value is the computed metric which was compared with threshold.
       */
      bool
      overlapAgreement (double threshold, const geometry::RotationMatrix& Rfixed, double& value);


    private:
      AbstractPlanarSegment::Ptr p1_1_;
      AbstractPlanarSegment::Ptr p1_2_;
      AbstractPlanarSegment::Ptr p2_1_;
      AbstractPlanarSegment::Ptr p2_2_;
  };
}


#endif // TWO_PAIRS_TESTER_H
