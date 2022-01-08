#ifndef REGION_GROWING_SEGMENTATION_PARAMETERS_H_
#define REGION_GROWING_SEGMENTATION_PARAMETERS_H_
namespace tams
{
  struct RegionGrowingSegmentationParameters
  {
    int sliding_window_size;
    double max_neighbor_dis;
    double max_point2plane_dis;
    double max_angle_difference;
    double max_segment_mse;
    double max_local_mse;
    double max_seed_mse;
    int min_segment_size;
    int nearest_neighbor_size;
    RegionGrowingSegmentationParameters():
      sliding_window_size (0), max_neighbor_dis (0.0), max_point2plane_dis (0.0),
      max_angle_difference (0.0), max_segment_mse (0.0), max_local_mse (0.0),
      max_seed_mse (0.0), min_segment_size (0), nearest_neighbor_size (0)
    {
    }
  };
}
#endif
