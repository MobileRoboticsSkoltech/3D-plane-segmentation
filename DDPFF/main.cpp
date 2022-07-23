#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <rep/DDPFF.h>

#include "globals/Config.h"

void read_pcd(pointBuffer_t & points_ptr, const char * pcd_path) {
    std::ifstream infile(pcd_path);

    if (!infile.is_open()) {
        throw std::runtime_error("Input file (cloud) not found!");
    }

    std::string line;
    // read header
    std::getline(infile, line);
    if (line != "ply") {
        throw std::runtime_error("Wrong input format: only PLY accepted, but '" + line + "' was provided!");
    }
    // skip header
    while (line != "end_header") {
        std::getline(infile, line);
    }

    // read data
    int i = 0;
    while (std::getline(infile, line))
    {
        std::istringstream iss(line);
        real_t x, y, z;
        iss >> x >> y >> z;
        Vec3 tmp;
        tmp << x, y, z;
        points_ptr[i] = tmp;
        i++;
    }
    std::cout << "Parsed point cloud with point: " << i << std::endl;
}

void save_planes(const std::vector<PlanePointNormal> & planes) {
    std::ofstream output("output/planes.txt");

    for (const auto & plane : planes) {
        for (auto inlier : plane.inliers) {
            output << inlier << " ";
        }
        output << std::endl;
    }
}

int main(int argc, char** argv) {
    const char* pcd_name = argv[1];
    const char* cfg_name = argv[2];

    config.read_ini(("input/" + std::string(cfg_name)).c_str());
   
    auto ddpff = new DDPFF();
    ddpff->init();

    auto * points = new pointBuffer_t();
    auto * colors = new colorBuffer_t();
    auto * depth = new depthBuffer_t();
    read_pcd(*points, ("input/" + std::string(pcd_name)).c_str());
    ddpff->setBuffers(points, colors, depth);

    ddpff->compute();
    const std::vector<PlanePointNormal> & result = ddpff->getPlanes();
    for (const auto& plane : result) {
        std::cout << "Plane with " << plane.count << " points detected!" << std::endl;
    }
    std::cout << "Totally detected " << result.size() << " planes" << std::endl;
    save_planes(result);
    return 0;
}
