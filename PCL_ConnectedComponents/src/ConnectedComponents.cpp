#include "ConnectedComponents.h"

ConnectedComponents::ConnectedComponents(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud) : cloud(std::move(cloud))
{
}

double ConnectedComponents::SegmentCloud(std::vector<pcl::PointIndices>& clusters) {
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud(new pcl::PointCloud<pcl::Normal>);
    ne_.setInputCloud(cloud);
    ne_.compute(*normal_cloud);

    std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA>>> regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    mps_.setInputNormals(normal_cloud);
    mps_.setInputCloud(cloud);
    mps_.segmentAndRefine(regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    if (!regions.empty()) {
        std::shared_ptr<std::set<std::uint32_t>> plane_labels(new std::set<uint32_t>);
        for (size_t i = 0; i < label_indices.size(); i++) {
            if (label_indices[i].indices.size() > 10000) {
                plane_labels->insert(i);
            }
        }
        pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Label>::Ptr euclidean_cluster_comparator_(
                new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Label>());
        euclidean_cluster_comparator_->setInputCloud(cloud);
        euclidean_cluster_comparator_->setLabels(labels);
        euclidean_cluster_comparator_->setExcludeLabels(plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold(0.01f, false);
        pcl::PointCloud<pcl::Label> euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label> euclidean_segmentation(
                euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud(cloud);
        clock_t start = clock();
        euclidean_segmentation.segment(euclidean_labels, euclidean_label_indices);
        clock_t stop = clock();
        for (auto & euclidean_label_indice : euclidean_label_indices) {
            if (euclidean_label_indice.indices.size() > 1000) {
                clusters.push_back(euclidean_label_indice);
            }
        }
        return (double)(stop - start)/(CLOCKS_PER_SEC/1000);
    }
    return 0;
}