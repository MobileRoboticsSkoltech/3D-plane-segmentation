#include "planar_segments_matcher/one_pair_tester.h"

using namespace tams;

OnePairTester::OnePairTester()
{

}

OnePairTester::OnePairTester(AbstractPlanarSegment::Ptr ps_1, AbstractPlanarSegment::Ptr ps_2):
  ps_1_(ps_1), ps_2_(ps_2)
{

}

bool
OnePairTester::areaSimilarityAgreement(double threshold, double &value)
{
  bool ret = false;

  double max_area = std::max(ps_1_->getArea(), ps_2_->getArea());
  value = fabs(ps_1_->getArea() - ps_2_->getArea()) / max_area;

  if (value <= threshold)
    ret = true;

  return ret;
}

bool
OnePairTester::areParallel(double threshold, double &value)
{
  bool ret = false;

  Eigen::Vector3d n1 = ps_1_->getNormal();
  Eigen::Vector3d n2 = ps_2_->getNormal();

  value = fabs(n1.dot(n2));

  if (value > threshold)
    ret = true;

  return ret;
}

bool
OnePairTester::areNonparallel(double min_angle,
                              double max_angle,
                              double &value)
{
  bool ret = false;

  if (min_angle < 0 || max_angle < 0 || max_angle < min_angle)
  {
    std::cerr << "please provide two positive values which satisfy: min_angle < max_angle."
              << "Example: min_angle = 30 and max_angle = 150." << std::endl;
  }

  Eigen::Vector3d n1 = ps_1_->getNormal();
  Eigen::Vector3d n2 = ps_2_->getNormal();
  double dot_product = n1.dot(n2);

  value = acos(dot_product) * 180.0 / M_PI;
  if (value <= max_angle && value >= min_angle)
    ret = true;

  return ret;
}

bool
OnePairTester::consistentWithRotation(double threshold,
                                      const Eigen::Matrix3d rotation_2_1,
                                      double &value)
{
  bool ret = false;

  Eigen::Vector3d n1 = ps_1_->getNormal();
  Eigen::Vector3d n2 = rotation_2_1 * ps_2_->getNormal();

  value = fabs(n1.dot(n2));

  if (value > threshold)
    ret = true;

  return ret;
}


bool OnePairTester::consistentWithTranslation(double threshold,
                                              const Eigen::Vector3d translation_2_1,
                                              const Eigen::Matrix3d cov_translation_2_1,
                                              double &value)
{

}

bool OnePairTester::odometryRotationAgreement()
{
  //Todo: read related paper and add this test.
}

bool OnePairTester::odometryTranslationAgreement()
{
  //Todo: read related paper and add this test.
}












OnePairTester::~OnePairTester()
{

}

