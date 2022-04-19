#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing.h>

class RegionGrowing {
public:
    double getClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<pcl::PointIndices>& clusters);
};
