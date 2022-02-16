#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include "src/RegionGrowing.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr readPointCloud(std::string pathToPCD) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile <pcl::PointXYZ> (pathToPCD, *pointCloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return pointCloud;
    }

    return pointCloud;
}

void writeClustersInDataFolder(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<pcl::PointIndices> clusters) {
    pcl::PCDWriter writer;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
            cloud_cluster->push_back ((*pointCloud)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "../generatedClusters/cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
}

int main() {
    std::string pathToPCD = "../data/0000_s.pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPointCloud(pathToPCD);

    RegionGrowing regionGrowing;
    std::vector<pcl::PointIndices> clusters = regionGrowing.getClusters(cloud);

    writeClustersInDataFolder(cloud, clusters);

    return 0;
}