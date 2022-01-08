#include "utils/tum.h"

#include <QFile>
#include <stdexcept>
#include <QTextStream>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <QProcess>
#include <opencv2/imgcodecs.hpp>
#include <QDebug>
#include <QFileInfo>

namespace tum
{
std::string extract(const std::string& sourceArchive, const std::string& targetDir) {
    std::filesystem::path sourcePath(sourceArchive);

    if (!std::filesystem::exists(sourcePath)) {
        throw std::runtime_error("Archive does not exist!");
    }

    QFileInfo qPath(sourceArchive.c_str());
    if (qPath.completeSuffix() != "tgz" && qPath.completeSuffix() != "tar.gz") {
        throw std::runtime_error(std::string("Unknown extension: ").append(qPath.completeSuffix().toStdString()));
    }

    std::filesystem::path targetPath(targetDir);
    if (!std::filesystem::exists(targetPath)) {
        std::filesystem::create_directory(targetPath);
    }

    if (!std::filesystem::is_directory(targetPath)) {
        throw std::runtime_error(std::string("Archive extraction directory is not valid: ").append(targetPath));
    }

    auto workingPath = targetPath / sourcePath.stem();

    if (std::filesystem::exists(workingPath)) {
        return workingPath;
    }

    QString program = "tar";
    QStringList arguments;
    arguments << "-zxf" << sourceArchive.c_str() << "-C" << targetDir.c_str();
    QProcess extractor;
    extractor.start(program, arguments);
    if (!extractor.waitForFinished()) {
        throw std::runtime_error("Error during archive extraction");
    }

    return workingPath;
}

void skipLines(std::istream& pStream, size_t pLines)
{
    std::string s;
    for (; pLines; --pLines)
        std::getline(pStream, s);
}

template <class CONTAINER>
typename CONTAINER::const_iterator find_closest(
    const CONTAINER& data, const typename CONTAINER::key_type& searchkey)
{
    typename CONTAINER::const_iterator upper = data.lower_bound(searchkey);
    if (upper == data.begin() || upper->first == searchkey) return upper;
    typename CONTAINER::const_iterator lower = upper;
    --lower;
    if (upper == data.end() ||
        (searchkey - lower->first) < (upper->first - searchkey))
        return lower;
    return upper;
}

DataExtractor::DataExtractor(const std::string &archivePath, const std::string &extractDir) :
    accurate(false),
    archivePath(archivePath),
    extractDir(extractDir)
{
    init();
}

DataExtractor::~DataExtractor()
{
    std::filesystem::remove_all(workingDir);
}

dataIterator_ptr_t DataExtractor::createIterator()
{
    return std::make_unique<DataIterator>(*this);
}

void tum::DataExtractor::extractAndParse()
{
    workingDir = extract(archivePath, extractDir);

    const std::string depthPath = workingDir + "/depth.txt";
    const std::string rgbPath = workingDir + "/rgb.txt";

    if (std::filesystem::exists(rgbPath))
    {
        std::ifstream inputFile(rgbPath);
        skipLines(inputFile, 3); // Skip the first three lines; these are comments
        real_t time;
        std::string filename;
        while (inputFile >> time >> filename) {
            rgbData[time] = filename;
        }
    }

    if (std::filesystem::exists(depthPath))
    {
        std::ifstream inputFile(depthPath);
        skipLines(inputFile, 3); // Skip the first three lines; these are comments
        real_t time;
        std::string filename;
        while (inputFile >> time >> filename) {
            depthData[time] = filename;
        }
    }

}

void tum::DataExtractor::synchronize()
{
    for (std::map<double, std::string>::const_iterator rgbData_it = rgbData.begin(); rgbData_it != rgbData.end(); ++rgbData_it++) {

        std::map<double, std::string>::const_iterator depthData_it = find_closest(depthData, rgbData_it->first);
        const double At_depth = std::abs(rgbData_it->first - depthData_it->first);
        if (accurate && At_depth > (1. / KINECT_FPS) * .5)
        {
            qWarning() << "Discarding depth observation for being too separated from "
                    "RGB timestamps: "
                 << At_depth * 1e3 << " ms\n";
            continue;
        }

        SynchronizedDataContainer data;
        data.rgbPath = rgbData_it->second;
        data.depthPath = depthData_it->second;

        synchronizedData.push_back(data);
    }
}

void DataExtractor::init()
{
    extractAndParse();

    qInfo() << "Parsed: " << depthData.size() << " / " << rgbData.size() << " / "
              << accData.size() << " / " << gtData.size() << " depth/rgb/acc/gt entries.\n";

    synchronize();

    qInfo() << synchronizedData.size() << " synchronized data frames formed \n";
}

void loadColor(const std::string &rgbPath, colorBuffer_t &colorBuffer) {
    if (!std::filesystem::exists(rgbPath)) {
        throw std::runtime_error(std::string("RGB image missing: ") + rgbPath);
    }

    cv::Mat rgbImage = cv::imread(rgbPath, cv::IMREAD_COLOR);

    for (size_t r = 0; r < IMAGE_HEIGHT; r++) {
        for (size_t c = 0; c < IMAGE_WIDTH; c++) {
            const size_t index = r * IMAGE_WIDTH + c;
            const cv::Vec3b &color = rgbImage.at<cv::Vec3b>(r,c);
            colorBuffer[index].x() = color(2); colorBuffer[index].y() = color(1); colorBuffer[index].z() = color(0);
        }
    }
}

void loadPointCloud(const std::string &depthPath, pointBuffer_t &pointBuffer, depthBuffer_t &depthBuffer) {
    static const real_t FY = 525.0;
    static const real_t FZ = 525.0;
    static const real_t CX = 319.5;
    static const real_t CZ = 239.5;
    static const real_t FACTOR = 5000.0;

    if (!std::filesystem::exists(depthPath)) {
        throw std::runtime_error(std::string("Depth image missing: ") + depthPath);
    }

    cv::Mat depthImage = cv::imread(depthPath, cv::IMREAD_ANYDEPTH);
    for (size_t r = 0; r < IMAGE_HEIGHT; r++) {
        for (size_t c = 0; c < IMAGE_WIDTH; c++) {
            const size_t index = r * IMAGE_WIDTH + c;
            const depth_t &depth = depthImage.at<depth_t>(r,c);
            depthBuffer[index] = depth;
            Vec3& point = pointBuffer[index];
            point.x() = depth / FACTOR;
            point.y() = (CX - c) * point.x() / FY;
            point.z() = (CZ - r) * point.x() / FZ;
        }
    }
}

DataIterator::DataIterator(DataExtractor &data) : data(data) {}

bool DataIterator::hasNext()
{
    return index < data.synchronizedData.size();
}

void DataIterator::loadNext(pointBuffer_t &pointBuffer, colorBuffer_t &colorBuffer, depthBuffer_t &depthBuffer)
{
    std::filesystem::path workingDirPath(data.workingDir);
    const DataExtractor::SynchronizedDataContainer &frame = data.synchronizedData[index];
    const auto &rgbPath = workingDirPath / frame.rgbPath;
    const auto &depthPath = workingDirPath / frame.depthPath;

    loadColor(rgbPath, colorBuffer);
    loadPointCloud(depthPath, pointBuffer, depthBuffer);

    index++;
}

}
