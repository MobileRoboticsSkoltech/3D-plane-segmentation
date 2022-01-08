#ifndef SEGCOMP_H
#define SEGCOMP_H


#include "globals/constants.h"

#include <string>
#include <memory>
#include <map>
#include <vector>

/*
 * Load and manipulate the segcomp ABW structured light depth images dataset.
*/
namespace segcomp
{

class DataIterator;

using dataIterator_ptr_t = std::unique_ptr<DataIterator>;

/*
 * Unzip and extract data from the archive.
*/
class DataExtractor {

public:

    /*
     * Takes two arguments:
     * 1. The path to the archive.
     * 2. The path to the extraction directory. If this is not supplied, the default is `/tmp/segcomp`.
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
     * Holds the paths to the depth, intensity and gt segmentation images for each frame.
    */
    struct DataContainer {
        std::string rangePath, intensityPath, gtPath;
    };

    const std::string archivePath;

    /* `workingDir` is the path of the extracted archive. */
    std::string extractionDir, workingDir;

    /* List of data frames. */
    std::vector<DataContainer> sortedData;

    /*
     * Initialize the data extractor. Performs the following steps:
     * 1. Extract the archive.
     * 2. Build a list of synchronized frames.
    */
    void init();

    inline static const std::string DEFAULT_EXTRACT_PATH = "/tmp/segcomp/";

    void extractAndParse();

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
     * 1. An intensity image.
     * 2. A depth image.
     * 3. A point cloud constructed from the depth image.
     * 5. Ground truth of the segmented planes in the image.
    */
    void loadNext(pointBuffer_t& pointBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer);

private:

    DataExtractor& data;

    size_t index = 0;
};
}


#endif // SEGCOMP_H
