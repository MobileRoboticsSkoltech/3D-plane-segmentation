#include <iostream>
#include <time.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include "src/ConnectedComponents.h"
#include <experimental/filesystem>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr readPointCloud(std::string pathToRGB) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    if (pcl::io::loadPCDFile <pcl::PointXYZRGBA> (pathToRGB, *pointCloud) == -1) {
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

void writeClustersInDataFolder(std::string pathToPCD, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloud, std::vector<pcl::PointIndices>& clusters) {
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
    for (auto const& pathToRGB : std::experimental::filesystem::directory_iterator{argv[1]}) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = readPointCloud(pathToRGB.path().string());
        ConnectedComponents connectedComponentsAlgorithm(cloud);
        std::vector<pcl::PointIndices> clusters;
        
        double comp_time = connectedComponentsAlgorithm.SegmentCloud(clusters);
        

        writeClustersInDataFolder(pathToRGB.path().string(), cloud, clusters);

        std::ofstream ex_time;
        ex_time.open ("./output/" + getFileName(pathToRGB.path().string()) + "/" +"ex_time.txt");
        ex_time << "Elapsed(ms)=" << comp_time << std::endl;
        ex_time.close();
    }
    return 0;
}