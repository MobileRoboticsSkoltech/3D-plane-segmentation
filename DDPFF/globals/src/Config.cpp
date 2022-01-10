#include "globals/Config.h"
#include "globals/constants.h"

#include <iostream>

// The global config object contains application wide configuration variables.
// Basically, the config object is a globally accessible struct with public
// read and write access for everyone (not thread safe!). Yes, even if this
// violates common programming paradigms, it focuses on simplicity of use.
// Just include the config header anywhere and use config.blabla to access a
// configuration parameter. Typically you will not want to write any parameters
// during runtime, only the config slider widget wants to do so.
// Similar to the State object, the Config object provides some basic reflection
// capabilities so that config sliders can be automatically generated for the gui.
// This is also used to automatically generate a config file that can be saved and
// loaded to preserve the config variables.
// All config variables are declared in this central place. Declare them in the
// config.h header, initialize them in the constructor and optionally register
// them in the init() method if you want a slider to be created and the config
// variable to be saved in the file. Every registered config variable gets a name
// and a slider factor assigned that determines the sensitivity of the slider.
// The config object also supports save() and load() functions that serializes
// and unserializes the variable contents in a hand editable text file. The save
// and load functions take a robot name as an argument to support different config
// sets for different robots.

Config config;

Config::Config()
{
    systemIterationTime = 0.05;
    debugLevel = -1;
    bufferSize = 10;
    sampleFactor = 1;

    // scene camera parameters
    sceneCameraRoll = 0;
    sceneCameraPitch = 0.80;
    sceneCameraYaw = 0;
    sceneCameraX = 0;
    sceneCameraY = 0;
    sceneCameraZ = 2;
    sceneRadius = 10;

    // flood fill parameters
    pointThresholdFloodFill_min = 0.01;
    pointThresholdFloodFill_max = 0.1;
    planeThresholdFloodFill_flood = 0.01;
    planeThresholdFloodFill_merge = 0.01;
    planeThresholdFloodFill_flood_max = 0.01;
    planeThresholdFloodFill_merge_max = 0.1;
    angleThresholdFloodFill = 0.1;
    angleThresholdFloodFill_max = 0.2;
    minPlaneSize = 100; // pixels
    normalSampleDistance_min = 5.0f;
    normalSampleDistance_max = 100.0f;
    c_plane = 1;
    c_plane_merge = 1;
    c_point = 2;
    c_range = 1.1;
    c_angle = 1;
}

// The init() method should be called after construction.
// Here, all config variables are registered to build a descriptor meta
// structure that allows index and key based access to their values.
// If you don't want to see a certain member on the gui, there is no
// need to register it.
void Config::init()
{
}
