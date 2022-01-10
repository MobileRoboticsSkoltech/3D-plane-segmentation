#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <rep/DDPFF.h>

void read_pcd(pointBuffer_t & points_ptr) {
    std::ifstream infile("input/result.ply");

    if (!infile.is_open()) {
        throw std::runtime_error("Input file not found!");
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

int main() {
    std::cout << "HI" << std::endl;

    auto ddpff = new DDPFF();
    ddpff->init();

    auto * points = new pointBuffer_t();
    auto * colors = new colorBuffer_t();
    auto * depth = new depthBuffer_t();
    read_pcd(*points);
    ddpff->setBuffers(points, colors, depth);

    ddpff->compute();
    const std::vector<PlanePointNormal> & result = ddpff->getPlanes();
    for (const auto& plane : result) {
        std::cout << plane.count << std::endl;
    }
    std::cout << result.size() << std::endl;
    save_planes(result);
    return 0;
}
