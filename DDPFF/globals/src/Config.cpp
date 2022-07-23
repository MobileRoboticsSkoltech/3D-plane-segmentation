#include "globals/Config.h"
#include "globals/constants.h"

#include <iostream>
#include <fstream>
#include <string>

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
    sampleFactor = 1;

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

void Config::read_ini(const char* cfg_path){
    std::ifstream input(cfg_path);

    std::string line;
    while (std::getline(input, line)){
    	if (line[0] == '[' || line[0] == '\n' || line.empty()) continue;
        std::string param_name = line.substr(0, line.find('='));
        real_t value = std::stod(line.substr(line.find('=') + 1));
        if (param_name == "floodFill.pointThreshold_min") config.pointThresholdFloodFill_min = value;
        else if (param_name == "floodFill.pointThreshold_max") config.pointThresholdFloodFill_max = value;
        else if (param_name == "floodFill.planeThreshold_flood") config.planeThresholdFloodFill_flood = value;
        else if (param_name == "floodFill.planeThreshold_merge") config.planeThresholdFloodFill_merge = value;
        else if (param_name == "floodFill.planeThreshold_flood_max") config.planeThresholdFloodFill_flood_max = value;
        else if (param_name == "floodFill.planeThreshold_merge_max") config.planeThresholdFloodFill_merge_max = value;
        else if (param_name == "floodFill.angleThresholdFloodFill") config.angleThresholdFloodFill = value;
        else if (param_name == "floodFill.angleThresholdFloodFill_max") config.angleThresholdFloodFill_max = value;
        else if (param_name == "floodFill.minPlaneSize") config.minPlaneSize = value;
        else if (param_name == "floodFill.normalSampleDistance_min") config.normalSampleDistance_min = value;
        else if (param_name == "floodFill.normalSampleDistance_max") config.normalSampleDistance_max = value;
        else if (param_name == "floodFill.c_plane") config.c_plane = value;
        else if (param_name == "floodFill.c_plane_merge") config.c_plane_merge = value;
        else if (param_name == "floodFill.c_point") config.c_point = value;
        else if (param_name == "floodFill.c_angle") config.c_angle = value;
        else if (param_name == "floodFill.c_range") config.c_range = value;
    }
}
