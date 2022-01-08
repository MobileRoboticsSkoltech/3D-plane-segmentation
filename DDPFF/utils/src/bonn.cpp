#include "utils/bonn.h"

#include <QFile>
#include <stdexcept>
#include <QTextStream>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <QProcess>
#include <QDebug>
#include <stdio.h>
#if __has_include(<pcl/io/pcd_io.h>)
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #define PCL_FOUND
#endif

#define RAS_MAGIC 0x59a66a95 /* rasterfile magic number */

namespace bonn
{
std::string extract(const std::string& sourceArchive, const std::string& targetDir) {
    std::filesystem::path sourcePath(sourceArchive);

    if (!std::filesystem::exists(sourcePath)) {
        throw std::runtime_error("Archive does not exist!");
    }

    if (sourcePath.extension() != ".zip") {
        throw std::runtime_error(std::string("Unknown extension: ").append(sourcePath.extension()));
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

    std::filesystem::create_directory(workingPath);

    QString program = "unzip";
    QStringList arguments;
    arguments << sourceArchive.c_str() << "-d" << workingPath.c_str();
    QProcess extractor;
    extractor.start(program, arguments);
    if (!extractor.waitForFinished()) {
        qCritical() << "Error during archive extraction:" << extractor.errorString();
        throw std::runtime_error("Error during archive extraction");
    }

    return workingPath;
}


DataExtractor::DataExtractor(const std::string &archivePath, const std::string &extractDir) :
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

void bonn::DataExtractor::extractAndParse()
{

    workingDir = extract(archivePath, extractDir);
    std::string pcdExt(".pcd");
    std::string gtExt(".gt-seg");

    std::vector<std::string> pcdFiles, gtFiles;

    // Gather pcd
    for (auto &p: std::filesystem::directory_iterator(workingDir)) {
        if (p.path().extension() == pcdExt) {
            pcdFiles.push_back(p.path());
        }
    }

    // Gather gt
    for (auto &p: std::filesystem::directory_iterator(workingDir)) {
        if (p.path().extension() == gtExt) {
            gtFiles.push_back(p.path());
        }
    }

    assert(pcdFiles.size() == gtFiles.size());

    // Sort the file names
    std::sort(pcdFiles.begin(), pcdFiles.end());
    std::sort(gtFiles.begin(), gtFiles.end());

    sortedData.resize(pcdFiles.size());

    // Put the matching files in the data container
    for (size_t i = 0; i < sortedData.size(); i++) {
        sortedData[i].pcdPath = pcdFiles[i];
        sortedData[i].gtPath = gtFiles[i];
    }

}


void DataExtractor::init()
{
    extractAndParse();

    qInfo() << "Extracted: " << sortedData.size() << " point cloud and ground truth frames";

}

bool loadGT(const std::string &gtPath, planeBuffer_t &planeBuffer) {
    if (!std::filesystem::exists(gtPath)) {
        throw std::runtime_error(std::string("Ground Truth missing: ") + gtPath);
    }

    int RasterHeader[8];
    FILE *fpt;
    static unsigned char *GTImage;
    static bool allocated = false;

    if (!allocated)
    {
        if ((GTImage=(unsigned char *)calloc(NUMBER_OF_POINTS, sizeof(unsigned char))) == NULL)
        {
            qCritical() << "Could not allocate space for array GTImage.";
            return false;
        } else {
            allocated = true;
        }
    }

    if ((fpt = fopen(gtPath.c_str(), "r")) == NULL) {
        qWarning() << "Couldn't open" << gtPath.c_str() << " for reading";
    }

    fread(RasterHeader, sizeof(int), 8, fpt); // read header

    fread(GTImage, sizeof(unsigned char), NUMBER_OF_POINTS, fpt); // read ground truth image

    fclose(fpt); // close file

    for (size_t i = 0; i < NUMBER_OF_POINTS; i++) {
        planeBuffer[i] = GTImage[i];
    }

    return true;

}

bool loadPointCloud(const std::string &pcdPath, pointBuffer_t &pointBuffer, depthBuffer_t &depthBuffer, colorBuffer_t &colorBuffer) {
#ifdef PCL_FOUND
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdPath, *cloud) == -1) //* load the file
    {
      qWarning() <<"Couldn't read file:" << pcdPath.c_str();
      return false;
    }

    if ((cloud->width * cloud->height) != NUMBER_OF_POINTS) {
        qWarning() << "Invalid cloud size:" << (cloud->width * cloud->height);
        return false;
    }

    for (int i = 0; i < NUMBER_OF_POINTS; i++)
    {
        const auto &point = (*cloud)[i];

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            pointBuffer[i].x() = 0;
            pointBuffer[i].y() = 0;
            pointBuffer[i].z() = 0;
            colorBuffer[i].x() = 0;
            colorBuffer[i].y() = 0;
            colorBuffer[i].z() = 0;
            depthBuffer[i] = 0;
            continue;
        }

        pointBuffer[i].x() = point.z;
        pointBuffer[i].y() = -point.x;
        pointBuffer[i].z() = -point.y;
        colorBuffer[i].x() = point.r;
        colorBuffer[i].y() = point.g;
        colorBuffer[i].z() = point.b;
        depthBuffer[i] = point.z;
    }

    return true;
#endif
}

DataIterator::DataIterator(DataExtractor &data) : data(data) {}

bool DataIterator::hasNext()
{
    return index < data.sortedData.size();
}

void DataIterator::loadNext(pointBuffer_t &pointBuffer, colorBuffer_t &colorBuffer, depthBuffer_t &depthBuffer, planeBuffer_t &planeBuffer)
{
#ifdef PCL_FOUND
    std::filesystem::path workingDirPath(data.workingDir);
    const DataExtractor::DataContainer &frame = data.sortedData[index];
    const auto &pcdPath = workingDirPath / frame.pcdPath;
    const auto &gtPath = workingDirPath / frame.gtPath;

    if (!loadPointCloud(pcdPath, pointBuffer, depthBuffer, colorBuffer) || !loadGT(gtPath, planeBuffer)) {
        pointBuffer.fill(Vec3(0,0,0));
        colorBuffer.fill(Pixel(0,0,0));
        depthBuffer.fill(0);
        planeBuffer.fill(0);
    }

    index++;
#endif
}

}
