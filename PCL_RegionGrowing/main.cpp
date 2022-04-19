#include <iostream>
#include <time.h>
#include <vector>
#include <experimental/filesystem>
#include <fstream>
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

std::string getFileName(std::string const &s) {
    char sep = '/';
    size_t i = s.rfind(sep, s.length());
    if (i != std::string::npos) {
        return(s.substr(i+1, s.length() - i - 5));
    }
    return("");
}

void writeClustersInDataFolder(std::string pathToPCD, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<pcl::PointIndices>& clusters) {
    std::vector<int> labels(pointCloud->size(), 0);
    int j = 1;

    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it) {
        for (const auto& idx : it->indices) {
            labels[idx] = j;
        }
        j++;
    }

    std::string newFolderName = getFileName(pathToPCD);
    std::experimental::filesystem::create_directories("./output/" + newFolderName);
    ofstream file("./output/" + newFolderName + "/" + newFolderName + ".txt");

    for (int label : labels) {
        file << std::to_string(label) + "\n";
    }

    file.close();
}

int main(int argc, char** argv) {
    for (auto const& pathToPCD : std::experimental::filesystem::directory_iterator{argv[1]})
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = readPointCloud(pathToPCD.path().string());

        RegionGrowing regionGrowing;
        std::vector<pcl::PointIndices> clusters;
        
        double comp_time = regionGrowing.getClusters(cloud, clusters);

        
        writeClustersInDataFolder(pathToPCD.path().string(), cloud, clusters);

        std::ofstream ex_time;
        ex_time.open ("./output/" + getFileName(pathToPCD.path().string()) + "/" +"ex_time.txt");
        ex_time << "Elapsed(ms)=" << comp_time << std::endl;
        ex_time.close();
    }

    return 0;
}