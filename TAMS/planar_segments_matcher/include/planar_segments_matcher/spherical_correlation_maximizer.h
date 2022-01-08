#ifndef SPHERICAL_CORRELATION_MAXIMIZER_H_
#define SPHERICAL_CORRELATION_MAXIMIZER_H_

#include <eigen3/Eigen/Core>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <map>
#include <utility>
#include <algorithm>
#include <time.h>

#include "planar_segments_matcher/correspondences.h"
#include "planar_segments_matcher/correspondence_pairs.h"
#include "common/abstract_planar_segment.h"
#include "planar_segments_matcher/matcher_parameters.h"
#include "planar_segments_matcher/one_pair_tester.h"
#include "planar_segments_matcher/two_pairs_tester.h"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation.h"

namespace tams
{
  /** The main class for planar segments based point cloud registration, the idea is to find
    the maximum spherical correlation between two point clouds.
    For detail see:
    Junhao Xiao, Benjamin Adler, Houxiang Zhang, "3D Point Cloud Registration Based on Planar Surfaces",
    2012 IEEE International Conference on Multisensor Fusion and Information Integration (MFI 2012), pp. 40-45, Hamburg, Germany, Sept., 2012.*/
  class SphericalCorrelationMaximizer
  {
    public:
      /** \brief Empty constructor. */
      SphericalCorrelationMaximizer();

      /** \brief Empty deconstructor. */
      ~SphericalCorrelationMaximizer();

    public:
      /** \brief Apply the OnePairTester on each L<->R pair, which results potential one pair correspondences.
        Then apply the TwoPairsTester on each (L1<->R1, L2<->R2), which results parallel and non-parallel pairs.*/
      enumerateSegmentPairsWithTesters(CorrespondencePairs &parallel_pairs, CorrespondencePairs &nonparallel_pairs);

      /** \brief Segments filter, the metric is the area of each planar segment.
        A segment will be filtered out if its area is smaller than the given threshold.*/

      void
      filterByArea(doublethreshold,
                   OctreeRGSegmentation::Ptr segmenter,
                   std::vector<AbstractPlanarSegment::Ptr> segments);

    private:
      Eigen::Vector3d translation_;
      Eigen::Matrix3d rotation_;

      OctreeRGSegmentation::Ptr segmenter_l_;
      OctreeRGSegmentation::Ptr segmenter_r_;

      std::vector<AbstractPlanarSegment::Ptr> segments_l_;
      std::vector<AbstractPlanarSegment::Ptr> segments_r_;

      MatcherParameters params_;

      /** The actual limits observed. These can be used to estimate thresholds later.*/
      PlaneCloudMatcherParameters _observedParams;
  };
}


#endif // SPHERICAL_CORRELATION_MAXIMIZER_H
