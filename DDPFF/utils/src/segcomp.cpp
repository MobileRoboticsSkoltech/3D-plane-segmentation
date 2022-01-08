#include "utils/segcomp.h"


#include "utils/segcomp.h"
#include "utils/eigenutils.h"

#include <QFile>
#include <stdexcept>
#include <QTextStream>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <QProcess>
#include <QDebug>
#include <stdio.h>
#include <QFileInfo>

#define RAS_MAGIC 0x59a66a95 /* rasterfile magic number */

namespace segcomp
{

std::string extractArchive(const std::string& sourceArchive, const std::string& targetDir) {
    std::filesystem::path sourcePath(sourceArchive);

    if (!std::filesystem::exists(sourcePath)) {
        throw std::runtime_error("Archive does not exist!");
    }

    QFileInfo qPath(sourceArchive.c_str());
    if (qPath.completeSuffix() != "abw.tar") {
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

    QString program = "tar";
    QStringList arguments;
    arguments << "-xf" << sourceArchive.c_str() << "-C" << workingPath.c_str();
    QProcess extractor;
    extractor.start(program, arguments);
    if (!extractor.waitForFinished()) {
        throw std::runtime_error("Error during archive extraction");
    }

    return workingPath;
}


/* Each image is compressed. Uncompress and put them in the same directory. */
void extractZFiles(const std::string &workingDir) {
    // Gather pcd
    QString program = "gunzip";
    QStringList arguments;
    QProcess extractor;
    for (auto &p: std::filesystem::directory_iterator(workingDir)) {
        if (p.path().extension() == ".Z" || p.path().extension() == ".z") {
            arguments.clear();
            arguments << p.path().c_str();
            extractor.start(program, arguments);
            if (!extractor.waitForFinished()) {
                throw std::runtime_error("Error during archive extraction");
            }
        }
    }
}


DataExtractor::DataExtractor(const std::string &archivePath, const std::string &extractionDir) :
    archivePath(archivePath),
    extractionDir(extractionDir)
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

void segcomp::DataExtractor::extractAndParse()
{

    workingDir = extractArchive(archivePath, extractionDir);
    extractZFiles(workingDir);

    std::string rangeExt(".range");
    std::string intensityExt(".intensity");
    std::string gtExt(".gt-seg");

    std::vector<std::string> rangeFiles, intensityFiles, gtFiles;

    // Gather file paths
    for (auto &p: std::filesystem::directory_iterator(workingDir)) {
        if (p.path().extension() == rangeExt) {
            rangeFiles.push_back(p.path());
        } else if (p.path().extension() == gtExt) {
            gtFiles.push_back(p.path());
        } else if (p.path().extension() == intensityExt) {
            intensityFiles.push_back(p.path());
        }
    }

    assert(rangeFiles.size() == gtFiles.size() && gtFiles.size() == intensityFiles.size());

    // Sort the file names
    std::sort(rangeFiles.begin(), rangeFiles.end());
    std::sort(intensityFiles.begin(), intensityFiles.end());
    std::sort(gtFiles.begin(), gtFiles.end());

    sortedData.resize(rangeFiles.size());

    // Put the matching files in the data container
    for (size_t i = 0; i < sortedData.size(); i++) {
        sortedData[i].rangePath = rangeFiles[i];
        sortedData[i].intensityPath = intensityFiles[i];
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

    int rasterHeader[8];
    FILE *fpt;
    static unsigned char *gtImage;
    static bool allocated = false;

    if (!allocated)
    {
        if ((gtImage=(unsigned char *)calloc(NUMBER_OF_POINTS, sizeof(unsigned char))) == NULL)
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

    fread(rasterHeader, sizeof(int), 8, fpt); // read header

    fread(gtImage, sizeof(unsigned char), NUMBER_OF_POINTS, fpt); // read ground truth image

    fclose(fpt); // close file

    for (size_t i = 0; i < NUMBER_OF_POINTS; i++) {
        planeBuffer[i] = gtImage[i];
    }

    return true;

}

bool loadRangeImage(const std::string &rangePath, pointBuffer_t &pointBuffer, depthBuffer_t &depthBuffer) {
    if (!std::filesystem::exists(rangePath)) {
        throw std::runtime_error(std::string("Range image missing: ") + rangePath);
    }

    int rasterHeader[8];
    FILE *fpt;
    static unsigned char *rangeImage;
    static bool allocated = false;

    if (!allocated)
    {
        if ((rangeImage=(unsigned char *)calloc(NUMBER_OF_POINTS, sizeof(unsigned char))) == NULL)
        {
            qCritical() << "Could not allocate space for array Range Image.";
            return false;
        } else {
            allocated = true;
        }
    }

    if ((fpt = fopen(rangePath.c_str(), "r")) == NULL) {
        qWarning() << "Couldn't open" << rangePath.c_str() << " for reading";
    }

    fread(rasterHeader, sizeof(int), 8, fpt); // read header

    fread(rangeImage, sizeof(unsigned char), NUMBER_OF_POINTS, fpt); // read range image

    fclose(fpt); // close file

    for (size_t i = 0; i < NUMBER_OF_POINTS; i++) {
        depthBuffer[i] = rangeImage[i];
    }

    static const real_t OFFSET = 771.016866;
    static const real_t SCAL = 0.791686;
    static const real_t F = -1586.072821;
    static const real_t C = 1.4508;

    for (int r=0; r < IMAGE_HEIGHT; r++) {
        for (int c=0; c < IMAGE_WIDTH; c++) {
            const size_t index = r * IMAGE_WIDTH + c;
            if (rangeImage[index] == 0)
            {
                pointBuffer[index].x() = 0; pointBuffer[index].y() = 0; pointBuffer[index].z() = 0;
                continue;
            }

            const real_t &X = (real_t)(c - 255) * ((real_t) rangeImage[index] / SCAL + OFFSET) / fabs(F);
            const real_t &Y = (real_t)(r - 255) / C * ((real_t)rangeImage[index] / SCAL + OFFSET) / fabs(F);
            const real_t &Z = (real_t)(rangeImage[index]) / SCAL;

            pointBuffer[index].x() = Z;
            pointBuffer[index].y() = -X;
            pointBuffer[index].z() = -Y;

            pointBuffer[index] /= 100.0; // cm to m
        }
    }

    return true;

}

bool loadIntensityImage(const std::string &intensityPath, colorBuffer_t &colorBuffer) {
    if (!std::filesystem::exists(intensityPath)) {
        throw std::runtime_error(std::string("Intensity image missing: ") + intensityPath);
    }

    int rasterHeader[8];
    FILE *fpt;
    static unsigned char *intensityImage;
    static bool allocated = false;

    if (!allocated)
    {
        if ((intensityImage=(unsigned char *)calloc(NUMBER_OF_POINTS, sizeof(unsigned char))) == NULL)
        {
            qCritical() << "Could not allocate space for array Intensity Image.";
            return false;
        } else {
            allocated = true;
        }
    }

    if ((fpt = fopen(intensityPath.c_str(), "r")) == NULL) {
        qWarning() << "Couldn't open" << intensityPath.c_str() << " for reading";
    }

    fread(rasterHeader, sizeof(int), 8, fpt); // read header

    fread(intensityImage, sizeof(unsigned char), NUMBER_OF_POINTS, fpt); // read ground truth image

    fclose(fpt); // close file

    for (size_t i = 0; i < NUMBER_OF_POINTS; i++) {
        colorBuffer[i].x() = colorBuffer[i].y() = colorBuffer[i].z() = intensityImage[i];
    }

    return true;
}

DataIterator::DataIterator(DataExtractor &data) : data(data) {}

bool DataIterator::hasNext()
{
    return index < data.sortedData.size();
}

void DataIterator::loadNext(pointBuffer_t &pointBuffer, colorBuffer_t &colorBuffer, depthBuffer_t &depthBuffer, planeBuffer_t &planeBuffer)
{
    std::filesystem::path workingDirPath(data.workingDir);
    const DataExtractor::DataContainer &frame = data.sortedData[index];
    const auto &rangePath = workingDirPath / frame.rangePath;
    const auto &intensityPath = workingDirPath / frame.intensityPath;
    const auto &gtPath = workingDirPath / frame.gtPath;

    bool rangeImageLoaded = loadRangeImage(rangePath, pointBuffer, depthBuffer);
    bool gtLoaded = loadGT(gtPath, planeBuffer);
    bool intensityImageLoaded = loadIntensityImage(intensityPath, colorBuffer);

    if (!rangeImageLoaded || !gtLoaded || !intensityImageLoaded) {
        pointBuffer.fill(Vec3(0,0,0));
        colorBuffer.fill(Pixel(0,0,0));
        depthBuffer.fill(0);
        planeBuffer.fill(0);
    }

    index++;
}

}
