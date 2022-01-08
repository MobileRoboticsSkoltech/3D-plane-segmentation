#ifndef OCTREE_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
#define OCTREE_REGION_GROWING_SEGMENTATION_PARAMETERS_H_
namespace tams
{
  struct OctreeRegionGrowingSegmentationParameters
  {
    double max_neighbor_dis;
    double max_point2plane_dis;
    double max_angle_difference;
    double max_segment_mse;
    double max_local_mse;
    double max_seed_mse;
    int min_segment_size;
    int sliding_sphere_size;
    int nearest_neighbor_size;
    bool downsampling;
    bool show_filtered_cloud;
    double downsampling_leafsize;
    int osr_mean_k;
    double osr_StddevMulThresh;
    OctreeRegionGrowingSegmentationParameters():
      max_neighbor_dis (0.0), max_point2plane_dis (0.0),
      max_angle_difference (0.0), max_segment_mse (0.0),
      max_local_mse (0.0), max_seed_mse (0.0),
      min_segment_size (0), sliding_sphere_size (0),
      nearest_neighbor_size (0), downsampling (false),
      show_filtered_cloud (false),
      downsampling_leafsize (0.0f), osr_mean_k (0),
      osr_StddevMulThresh (0.0f)
    {
    }
  };
}
#endif
