//
// Created by michael on 23.07.2022.
//

#include "write_labels.h"

void writeLabelsTable(const string &path, size_t height, size_t width, cv::Mat_<uchar> &data) {
    uchar *sCode;
    ofstream fout(path);

    for (int r = 0; r < height; r++) {
        sCode = data.ptr<uchar>(r);
        for (int c = 0; c < width; c++) {
            fout << +(*sCode);
            if (c != width - 1)
                fout << ',';
            sCode++;
        }
        fout << '\n';
    }
    fout.close();
}