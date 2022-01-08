#ifndef HYBRID_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
#define HYBRID_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
namespace tams
{
  struct HybridRGSegmentationParameters
  {
    double min_dot_product;
    double max_mass2plane_dis;
    double max_segment_mse;
    int min_segment_size;
    int subwindow_side_length;
    HybridRGSegmentationParameters():
      min_dot_product (0.0), max_mass2plane_dis (0.0), max_segment_mse (0.0), min_segment_size (0),
      subwindow_side_length (0)
    {
    }
  };
}
#endif
