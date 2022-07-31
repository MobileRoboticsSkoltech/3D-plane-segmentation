//
// Created by michael on 22.07.2022.
//

#pragma once

#include <fstream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;

void writeLabelsTable(const string &path, size_t height, size_t width, cv::Mat_<uchar> &data);
