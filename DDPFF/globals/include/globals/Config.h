#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <QList>
#include <QHash>
#include <QString>
#include <QFile>
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
    void save(QString name = "");
    void load(QString name = "");

    real_t& operator[](int i);
    real_t& operator()(int i);
    real_t& operator[](QString key);
    real_t& operator()(QString key);

private:

    real_t sink;

    // Registers a member variable for index based access.
    void registerMember(QString name, real_t* member, real_t sliderFactor)
    {
        memberNames << name;
        memberOffsets[name] = (quint64)member - (quint64)this;
        sliderFactors[name] = sliderFactor/100;
    }

    QHash<QString, ptrdiff_t> memberOffsets;

public:
    QList<QString> memberNames; // Contains the names of the members in the right order.
    QHash<QString, real_t> sliderFactors; // The factors of all explicitely registered config variables.
};

extern Config config;

#endif /* CONFIGURATION_H_ */
