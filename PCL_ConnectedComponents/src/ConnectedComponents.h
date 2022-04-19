#include <iostream>
#include <memory>
#include <utility>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

class ConnectedComponents {
private:
    pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne_;
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> mps_;

public:
    ConnectedComponents(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);
    double SegmentCloud(std::vector<pcl::PointIndices>& clusters);
};