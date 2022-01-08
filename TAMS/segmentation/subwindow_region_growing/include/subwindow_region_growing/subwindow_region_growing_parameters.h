#ifndef SUBWINDOW_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
#define SUBWINDOW_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
namespace tams
{
  struct SubwindowRGSegmentationParameters
  {
    double min_dot_product;
    double max_mass2plane_dis;
    double max_segment_mse;
    int subwindow_side_length;
    int min_segment_size;
    SubwindowRGSegmentationParameters():
      min_dot_product (0.0), max_mass2plane_dis (0.0), max_segment_mse (0.0),
      subwindow_side_length (0), min_segment_size (0)
    {
    }
  };
}
#endif
