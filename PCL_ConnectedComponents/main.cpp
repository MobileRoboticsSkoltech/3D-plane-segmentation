#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include "src/ConnectedComponents.hpp"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPointCloud(std::string pathToRGB) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile <pcl::PointXYZRGBA> (pathToRGB, *pointCloud) == -1) {
        std::cout << "Cloud reading failed." << std::endl;
        return pointCloud;
    }

    return pointCloud;
}

int main() {
    std::string pathToRGB = "../input/0.pcd";
    ConnectedComponents<pcl::PointXYZRGBA> connectedComponentsAlgorithm(readPointCloud(pathToRGB));
    pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType clusters = connectedComponentsAlgorithm.SegmentCloud();
    std::cout << clusters.size();
    return 0;
}