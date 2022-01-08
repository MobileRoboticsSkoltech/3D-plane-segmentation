#ifndef REGION_GROWING_SEGMENTATION_H_
#define REGION_GROWING_SEGMENTATION_H_
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigenvalues>
#include "common/planar_patch.h"
#include "region_growing_segmentation/region_growing_segmentation_parameters.h"

#include <sys/time.h>
#include <iostream>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <utility>
namespace tams
{
  using namespace std;
  using namespace Eigen;

  struct SlidingWindowItem
  {
    int index;
    double  mse;
    Vector3d normal;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SlidingWindowItem () :
      index (0), mse (0), normal (Vector3d::Zero())
    {
    }

    ~SlidingWindowItem ()
    {
    }

    bool
    operator< (const SlidingWindowItem& rhs) const
    {
      return mse < rhs.mse;
    }
  };

  template <typename PointT>
  class RGSegmentation : public pcl::PCLBase<PointT>
  {
  public:
    using pcl::PCLBase<PointT>::initCompute;
    using pcl::PCLBase<PointT>::deinitCompute;
    using pcl::PCLBase<PointT>::input_;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

    /** \brief Empty constructor.*/
    RGSegmentation():
      max_neighbor_dis_(0.0), max_point2plane_dis_(0.0),
      max_angle_difference_ (0.0), max_segment_mse_(0.0),
      max_local_mse_ (0.0), max_seed_mse_ (0.0),
      nearest_neighbor_size_ (0), min_segment_size_(0.0),
      sliding_window_size_ (0)
    {

    }

    /** \brief Compute the Euler distance of two given points.
     *
     * @param[in] point1 The first given point.
     * @param[in] point2 The second given point.
     * @return The Euler distance between point1 and point2.
     */
    double
    disPoint2Point(const PointT point1,const PointT point2)
    {
      return (point1.x - point2.x)*(point1.x - point2.x) +
             (point1.y - point2.y)*(point1.y - point2.y) +
             (point1.z - point2.z)*(point1.z - point2.z);
    }


    /** \brief Compute local normal and local mse for valid points.*/
    void
    slidingWindow(const int slding_window_size);


    /** \brief Investigating neighbor points of the current added point
     *
     * @param[in] x u position of the new added point
     * @param[in] y v position of the new added point
     */

    void investigate8Neighbors(const int x, const int y);
    void investigate8Neighbors(const int index);
    /** \brief Investigating neighbor points of the current added point
     *
     * @param[in] x u position of the new added point
     * @param[in] y v position of the new added point
     */
    void investigate24Neighbors(const int x, const int y);

    /** \brief Segment the input point cloud into planar patches.
     *
     * @param[out] output Planar patches in different colors.
     */
    void segmentation(CloudXYZRGB::Ptr &output)
    {
      if (!initCompute ())
      {
        return;
      }
      // Copy header at a minimum
      output_.header = input_->header;
      output_.sensor_origin_ = input_->sensor_origin_;
      output_.sensor_orientation_ = input_->sensor_orientation_;

      height_ = input_->height;
      width_ = input_->width;
      points_.resize(height_ * width_);
      for (int i = 0; i < height_ * width_; i++)
      {
        points_[i](0) = input_->points[i].x;
        points_[i](1) = input_->points[i].y;
        points_[i](2) = input_->points[i].z;
      }

      applySegmentation(output);
      deinitCompute();
    }

    void
    getSegments(PlanarSegment::StdVector &segments)
    {
      segments.clear();
      segments = planar_patches_;
    }
    /** \brief Compute square area for each segment.
     *
     */
    void
    computeSegmentsArea();
    void setParameters (RegionGrowingSegmentationParameters &parameters)
    {
      sliding_window_size_ = parameters.sliding_window_size;
      max_neighbor_dis_ = parameters.max_neighbor_dis * parameters.max_neighbor_dis;
      max_point2plane_dis_ = parameters.max_point2plane_dis;
      max_angle_difference_ = cos(parameters.max_angle_difference * M_PI / 180) ;
      max_segment_mse_ = parameters.max_segment_mse;
      max_local_mse_ = parameters.max_local_mse;
      max_seed_mse_ = parameters.max_seed_mse;
      nearest_neighbor_size_ = parameters.nearest_neighbor_size;
      min_segment_size_ = parameters.min_segment_size;
    }
    /** \brief Set random color to the detected big planar patches.
     * The colored planar patches will be put into cloud output->
     * @param output cloud with colored planar patches
     */
    void colorEncoding (CloudXYZRGB::Ptr &output);

    /**\brief Store the time for each planar segments.
     *
     */
    void saveTimes();

  private:

    /** \brief Segment the input cloud into big planar patches.
     *
     * @param output the output cloud with big planar patches
     */
    void
    applySegmentation (CloudXYZRGB::Ptr &output);

    double
    computeRightBottomArea(const int pos, PlanarSegment &segment);
    double
    computeLeftTopArea(const int pos, PlanarSegment &segment);


    void
    parameterUncertainty();

    void
    computeVolume ();


  private:
    /** \brief The segmentation name. */
    double  max_neighbor_dis_;
    double  max_point2plane_dis_;
    double  max_angle_difference_;
    double  max_segment_mse_;
    double  max_local_mse_;
    double  max_seed_mse_;
    int nearest_neighbor_size_;
    int min_segment_size_;
    int sliding_window_size_;
    vector<int> neighbor_points_;
    vector<int> remained_points_;
    PlanarSegment::StdVector planar_patches_;
    vector<Vector3d, aligned_allocator<Vector3d> > points_;
    int height_;
    int width_;
    bool * visited_;
    bool * valid_;
    bool * added_to_region_;
    bool * has_local_plane_;
    double  * local_mse_;
    vector<timeval> times;
    timeval tmp_time;
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > local_normals_;
    CloudXYZRGB output_;
    vector<SlidingWindowItem> sliding_windows_;
    size_t badpoints_num_;
    ofstream ofile_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
