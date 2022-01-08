#ifndef MATCHER_PARAMETERS_H_
#define MATCHER_PARAMETERS_H_

namespace tams
{
  struct MatcherParameters
  {
    bool visualization;
    bool odometry_consistent_test;
    double min_area;
    double merge_angle;
    double merge_dis;
    double max_area_diff;
    double unparallel_min_angle;
    double unparallel_max_angle;
    double side_rotation_ability;
    double max_translation_norm;
    double max_angle_diff;
    double max_bias_diff;

    MatcherParameters():
      visualization (false),
      merge_angle (0.0),
      merge_dis (0.0),
      max_area_diff (0.0),
      unparallel_min_angle (0.0),
      unparallel_max_angle (0.0),
      side_rotation_ability (0.0),
      max_translation_norm (0.0),
      max_angle_diff (0.0),
      max_bias_diff (0.0)
    {
    }
  };
}


#endif // MATCHER_PARAMETERS_H
