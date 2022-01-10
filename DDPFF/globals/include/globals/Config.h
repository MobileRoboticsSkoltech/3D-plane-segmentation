#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <stdint.h>

#include "globals/constants.h"

struct Config
{
    real_t systemIterationTime;
    real_t debugLevel;
    real_t bufferSize;

    int sampleFactor;

    // scene camera parameters
    real_t sceneCameraRoll;
    real_t sceneCameraPitch;
    real_t sceneCameraYaw;
    real_t sceneCameraX;
    real_t sceneCameraY;
    real_t sceneCameraZ;
    real_t sceneRadius;

    // flood fill parameters
    real_t pointThresholdFloodFill_min;
    real_t pointThresholdFloodFill_max;
    real_t planeThresholdFloodFill_flood;
    real_t planeThresholdFloodFill_merge;
    real_t planeThresholdFloodFill_flood_max;
    real_t planeThresholdFloodFill_merge_max;
    real_t angleThresholdFloodFill;
    real_t angleThresholdFloodFill_max;
    real_t normalSampleDistance_min;
    real_t normalSampleDistance_max;
    real_t c_plane;
    real_t c_plane_merge;
    real_t c_point;
    real_t c_angle;
    real_t c_range;

    // common for flood fill and ransac
    real_t minPlaneSize;

	Config();
    ~Config(){}

	void init();

private:
    real_t sink;
};

extern Config config;

#endif /* CONFIGURATION_H_ */
