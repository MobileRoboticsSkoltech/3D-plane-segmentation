#include <iostream>
#include <time.h>
#include <vector>
#include <experimental/filesystem>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/visualization/cloud_viewer.h>
#include "src/headers/subwindow_region_growing.h"
#include "src/headers/subwindow_region_growing_parameters.h"

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

void writeClustersInDataFolder(std::string pathToPCD, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, std::vector<std::vector<int>>& clusters) {
    std::vector<int> labels(pointCloud->size(), 0);
    int j = 1;

    for (std::vector<int> cluster : clusters) {
        for (const auto& idx : cluster) {
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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
        tams::SubwindowRGSegmentationParameters parameters;
        parameters.max_mass2plane_dis = 0.035;
        parameters.min_dot_product = 0.95;
        parameters.max_segment_mse = 0.0008;
        parameters.min_segment_size = 50;
        parameters.subwindow_side_length = 3;

        tams::SubwindowRGSegmentation segmentation;
        segmentation.setparameters(parameters);
        segmentation.setInput(cloud);

        clock_t start = clock();
        segmentation.preprocessing();
        std::vector<std::vector<int>> indices = segmentation.applySegmentation();
        clock_t stop = clock();
        std::ofstream ex_time;
        std::cout << pathToPCD.path().string() << std::endl;
        writeClustersInDataFolder(pathToPCD.path().string(), cloud, indices);
        ex_time.open ("./output/" + getFileName(pathToPCD.path().string()) + "/" +"ex_time.txt");
        ex_time << "Elapsed(ms)=" << (double)(stop - start)/(CLOCKS_PER_SEC/1000) << std::endl;
        ex_time.close();
    }
    return 0;
}