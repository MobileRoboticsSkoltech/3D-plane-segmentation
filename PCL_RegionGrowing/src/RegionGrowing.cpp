#include "RegionGrowing.h"

std::vector <pcl::PointIndices> RegionGrowing::getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud) {
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(pointCloud);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::IndicesPtr indices(new std::vector <int>);
    pcl::removeNaNFromPointCloud(*pointCloud, *indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(1000000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(pointCloud);
    reg.setIndices(indices);
    reg.setInputNormals(normals);

    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);

    return clusters;
}
