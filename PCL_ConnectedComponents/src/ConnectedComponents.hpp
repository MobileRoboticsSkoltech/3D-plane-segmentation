#include <iostream>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/common/time.h>

template <typename PointT>
class ConnectedComponents {
public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    CloudConstPtr cloud;
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne_;
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps_;

    ConnectedComponents(CloudConstPtr cloud) : cloud(cloud)
    {
    }
    std::vector<pcl::PointIndices> SegmentCloud(std::vector<pcl::PointIndices>& clusters);
};

template<typename PointT>
std::vector<pcl::PointIndices> ConnectedComponents<PointT>::SegmentCloud(std::vector<pcl::PointIndices>& clusters) {
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
    pcl::PointCloud<pcl::PointXYZRGB> aggregate_cloud;

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(cloud);
    ne_.compute(*normal_cloud);

    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(cloud);
    mps_.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    if (regions.size () > 0) {
        std::shared_ptr<std::set<std::uint32_t>> plane_labels(new std::set<uint32_t>);
        for (size_t i = 0; i < label_indices.size(); i++) {
            if (label_indices[i].indices.size() > 10000) {
                plane_labels->insert(i);
            }
        }
        typename pcl::EuclideanClusterComparator<PointT, pcl::Label>::Ptr euclidean_cluster_comparator_(
                new typename pcl::EuclideanClusterComparator<PointT, pcl::Label>());
        euclidean_cluster_comparator_->setInputCloud(cloud);
        euclidean_cluster_comparator_->setLabels(labels);
        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);

        pcl::PointCloud<pcl::Label> euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation(
                euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud(cloud);
        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);

        for (size_t i = 0; i < euclidean_label_indices.size(); i++) {
            if (euclidean_label_indices[i].indices.size() > 1000) {
                clusters.push_back(euclidean_label_indices[i]);
            }
        }
    }

    return clusters;
}