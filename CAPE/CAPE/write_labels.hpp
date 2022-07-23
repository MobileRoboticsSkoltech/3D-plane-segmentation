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

template<class T>
void writeLabelsTable(const string &path, size_t height, size_t width, cv::Mat_<T> &data) {
    T *sCode;
    ofstream fout(path);

    for (int r = 0; r < height; r++) {
        sCode = data.template ptr<T>(r);
        for (int c = 0; c < width; c++) {
            fout << static_cast<uint>(*sCode) << ',';
            sCode++;
        }
        fout << '\n';
    }
    fout.close();
}
