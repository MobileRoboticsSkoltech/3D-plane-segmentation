#include "utils/camerautils.h"

namespace utils {

    pointBuffer_t initializeUnitImage() {
        pointBuffer_t unitImage;

        // This is the cell size in x and y direction of an image one distance unit (here meters) away.
        // cellSizeX and cellSizeY should ideally be the same.
        double cellSizeX = 2 * tan(0.5 * CAMERA_OPENING_X * DEG_TO_RAD) / (double)IMAGE_WIDTH;
        double cellSizeY = 2 * tan(0.5 * CAMERA_OPENING_Y * DEG_TO_RAD) / (double)IMAGE_HEIGHT;

        // This is the center of the edge cells in x and y.
        double xMax = tan(0.5 * CAMERA_OPENING_X * DEG_TO_RAD) - cellSizeX * 0.5;
        double yMin = -tan(0.5 * CAMERA_OPENING_Y * DEG_TO_RAD) + cellSizeY * 0.5;

        // Loops through the image and sets the vector directions for every cell in the image.
        // The 0.001 is necessary if the resulting points are to have units in meters as the sensor provides data in mm.
        for (size_t y = 0; y < IMAGE_HEIGHT ; ++y)
        {
            for (size_t x = 0; x < IMAGE_WIDTH; ++x)
            {
                const size_t idx = y * IMAGE_WIDTH + x;
                // mm -> m
                unitImage[idx].x() = 0.001;
                unitImage[idx].y() = (xMax - x * cellSizeX) * 0.001;
                unitImage[idx].z() = (yMin + y * cellSizeY) * -0.001;
            }
        }

        return unitImage;
    }

    Transform3D makeCameraProjectionMatrix(){
        Transform3D out;

        out.matrix()(0,0) = (0.5 * IMAGE_WIDTH);
        out.matrix()(0,1) = (0.5 * IMAGE_HEIGHT);
        out.matrix()(0,2) = 1;
        out.matrix()(0,3) = 1;
        out.matrix()(1,0) = (-0.5 / tan(0.5 * CAMERA_OPENING_X * DEG_TO_RAD)) * IMAGE_WIDTH;
        out.matrix()(1,1) = 0;
        out.matrix()(1,2) = 0;
        out.matrix()(1,3) = 0;
        out.matrix()(2,0) = 0;
        out.matrix()(2,1) = (-0.5 / tan(0.5 * CAMERA_OPENING_Y * DEG_TO_RAD)) * IMAGE_HEIGHT;
        out.matrix()(2,2) = 0;
        out.matrix()(2,3) = 0;
        out.matrix()(3,0) = 0;
        out.matrix()(3,1) = 0;
        out.matrix()(3,2) = 0;
        out.matrix()(3,3) = 0;

        return Transform3D(out.matrix().transpose());
    }

    pointBuffer_t initializeNormalizedUnitImage()
    {
        pointBuffer_t normalizedUnitImage;

        for (size_t y = 0; y < IMAGE_HEIGHT ; ++y)
        {
            for (size_t x = 0; x < IMAGE_WIDTH; ++x)
            {
                const size_t idx = y * IMAGE_WIDTH + x;
                normalizedUnitImage[idx] = utils::unitImage[idx].normalized();
            }
        }

        return normalizedUnitImage;
    }

}
