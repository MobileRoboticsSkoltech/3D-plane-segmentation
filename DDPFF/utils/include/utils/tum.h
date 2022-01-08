#ifndef TUM_H
#define TUM_H

#include "globals/constants.h"

#include <string>
#include <memory>
#include <map>
#include <vector>

/*
 * Load and manipulate the TUM RGBD SLAM Dataset.
*/
namespace tum
{

class DataIterator;

using dataIterator_ptr_t = std::unique_ptr<DataIterator>;

/*
 * Unzip and extract data from the .tgz archive.
 * Inspired by: https://github.com/MRPT/mrpt/blob/master/samples/rgbd_dataset2rawlog/test.cpp
*/
class DataExtractor {

public:

    /*
     * Takes two arguments:
     * 1. The path to the .tgz archive.
     * 2. The path to the extraction directory. If this is not supplied, the default is `/tmp/tum`.
    */
    DataExtractor(const std::string& archivePath, const std::string& extractDir = DEFAULT_EXTRACT_PATH);

    /*
     * Deletes the extracted archive as cleanup.
    */
    ~DataExtractor();

    friend class DataIterator;

    /*
     * Iterator to the extracted data.
    */
    dataIterator_ptr_t createIterator();

private:

    /*
     * Holds the depth, accelerometer and ground truth data synchronized with the RGB frame.
    */
    struct SynchronizedDataContainer {
        std::string rgbPath;
        std::string depthPath;
    };

    const bool accurate;

    const std::string archivePath;

    /* `workingDir` is the path of the extracted archive. */
    std::string extractDir, workingDir;

    /* Raw data. */
    std::map<real_t, std::string> rgbData, depthData;
    std::map<real_t, std::vector<real_t>> accData, gtData;

    /* List of synchronized data frames. */
    std::vector<SynchronizedDataContainer> synchronizedData;

    /*
     * Initialize the data extractor. Performs the following steps:
     * 1. Extract the archive.
     * 2. Build a list of synchronized frames.
    */
    void init();

    inline static const std::string DEFAULT_EXTRACT_PATH = "/tmp/tum/";
    inline static const real_t KINECT_FPS = 30.0;

    void extractAndParse();

    /*
     * The timestamps of the RGB and depth data don't exactly match.
     * So we need to synchronize between the frames by finding the
     * timestamps that match the most.
     *
     * We assume the timestamps of the RGB frames as the root timestamps
     * and search for the closest ones in the rest of the frames.
    */
    void synchronize();
};

/*
 * An iterator over the extracted and synchronized data.
*/
class DataIterator {

public:

    DataIterator(DataExtractor& data);

    /*
     * Checks if more data frames are available.
    */
    bool hasNext();

    /*
     * Loads the next data frame and puts it in the supplied buffers.
     * A data frame consists of:
     * 1. A color image
     * 2. A depth image
     * 3. A point cloud constructed from the depth image.
    */
    void loadNext(pointBuffer_t& pointBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer);

private:

    DataExtractor& data;

    size_t index = 0;
};
}

#endif // TUM_H
