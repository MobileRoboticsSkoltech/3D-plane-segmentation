#include "planar_segments_matcher/spherical_correlation_maximizer.h"

namespace tams
{
  SphericalCorrelationMaximizer::SphericalCorrelationMaximizer()
  {

  }

  SphericalCorrelationMaximizer::~SphericalCorrelationMaximizer()
  {

  }

  SphericalCorrelationMaximizer::enumerateSegmentPairsWithTesters(CorrespondencePairs &parallel_pairs, CorrespondencePairs &nonparallel_pairs)
  {
    size_t Nl = segments_l_.size();
    size_t Nr = segments_r_.size();

    std::cerr << "there are " << Nl << " and " << Nr << " segments in the reference and new cloud." << std::endl;

    doublevalue;
    for (size_t oxL = 0; oxL < Nl; oxL++)
    {
      for (size_t oxR = 0; oxR < Nr; oxR++)
      {
        OnePairTester outerOnePairTester(segments_l_[oxL], segments_r_[oxR]);

        if (!outerOnePairTester.areaSimilarityAgreement(params_.max_area_diff, value))
        {
          continue;
        }

        if (params_.odometry_consistent_test)
        {
          if (!outerOnePairTester.odometryTranslationAgreement(params_.))
        }

      }
    }
  }

  void
  SphericalCorrelationMaximizer::filterByArea(doublethreshold,
                                              OctreeRGSegmentation::Ptr segmenter,
                                              std::vector<AbstractPlanarSegment::Ptr> segments)
  {

  }

}
