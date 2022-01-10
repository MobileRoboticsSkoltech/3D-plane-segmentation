#include "globals/Config.h"
#include "globals/constants.h"

//#include <QFile>
//#include <QTextStream>
//#include <QStringList>
//#include <QDebug>
//#include <QDir>
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
//    registerMember("systemIterationTime", &systemIterationTime, 1.0);
//    registerMember("debugLevel", &debugLevel, 100.0);
//    registerMember("bufferSize", &bufferSize, 1000.0);
//
//    registerMember("scene.cameraX", &sceneCameraX, 5);
//    registerMember("scene.cameraY", &sceneCameraY, 5);
//    registerMember("scene.cameraZ", &sceneCameraZ, 5);
//    registerMember("scene.cameraRoll", &sceneCameraRoll, 3.14);
//    registerMember("scene.cameraPitch", &sceneCameraPitch, 3.14);
//    registerMember("scene.cameraYaw", &sceneCameraYaw, 3.14);
//
//    registerMember("floodFill.pointThreshold_min", &pointThresholdFloodFill_min, 0.01);
//    registerMember("floodFill.pointThreshold_max", &pointThresholdFloodFill_max, 1.0);
//    registerMember("floodFill.planeThreshold_flood", &planeThresholdFloodFill_flood, 0.01);
//    registerMember("floodFill.planeThreshold_merge", &planeThresholdFloodFill_merge, 0.1);
//    registerMember("floodFill.planeThreshold_flood_max", &planeThresholdFloodFill_flood_max, 0.1);
//    registerMember("floodFill.planeThreshold_merge_max", &planeThresholdFloodFill_merge_max, 1.0);
//    registerMember("floodFill.angleThresholdFloodFill", &angleThresholdFloodFill, 0.1);
//    registerMember("floodFill.angleThresholdFloodFill_max", &angleThresholdFloodFill_max, 1.0);
//    registerMember("floodFill.minPlaneSize", &minPlaneSize, 1000);
//    registerMember("floodFill.normalSampleDistance_min", &normalSampleDistance_min, 100);
//    registerMember("floodFill.normalSampleDistance_max", &normalSampleDistance_max, 100);
//    registerMember("floodFill.c_plane", &c_plane, 10);
//    registerMember("floodFill.c_plane_merge", &c_plane_merge, 100);
//    registerMember("floodFill.c_point", &c_point, 10);
//    registerMember("floodFill.c_angle", &c_angle, 10);
//    registerMember("floodFill.c_range", &c_range, 10);
}

// Loads the config variables from the .conf file.
// Unregistered variables are ignored.
//void Config::load(QString name)
//{
//    if (name.isEmpty())
//        name = "config";
//
//    QFile file(CONFDIR + name + ".conf");
//	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
//	{
//		qDebug() << "Couldn't load config file" << file.fileName();
//		return;
//	}
//
//	QTextStream in(&file);
//	QString line;
//	QStringList list;
//	bool ok;
//	while (!in.atEnd())
//	{
//		line = in.readLine().trimmed();
//		list = line.split("=");
//		if (list.length() == 2 && !line.startsWith("//") && !line.startsWith("#"))
//		{
//			QString key = list[0].trimmed();
//            real_t value = list[1].trimmed().toDouble(&ok);
//			if (memberNames.contains(key))
//				this->operator[](key) = value;
//		}
//	}
//
//	file.close();
//}
//
//// Saves the config variables to the .conf file.
//void Config::save(QString name)
//{
//    if (name.isEmpty())
//        name = "config";
//
//    QDir dir;
//    if (!dir.exists(CONFDIR)) {
//        dir.mkpath(CONFDIR);
//    }
//
//    QFile file(CONFDIR + name + ".conf");
//	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
//	{
//		qDebug() << "Couldn't open config file" << file.fileName();
//		return;
//	}
//
//    QTextStream out(&file);
//    foreach (QString key, memberNames){
//        out << key << "=" << QString::number(this->operator[](key)) << "\n";
//    }
//	file.close();
//}
//
//// Returns a reference to the ith member of this object.
//real_t& Config::operator()(int i)
//{
//	return this->operator[](memberNames[i]);
//}
//
//// Returns a reference to the ith member of this object.
//real_t& Config::operator[](int i)
//{
//	return this->operator[](memberNames[i]);
//}
//
//// Returns a reference to the member that was registered with the given key.
//real_t& Config::operator()(QString key)
//{
//	return this->operator[](key);
//}
//
//// Returns a reference to the member that was registered with the given key.
//// If you try to access an unregistered member, you will get a useless reference to a black hole and a warning.
//real_t& Config::operator[](QString key)
//{
//	if (!memberNames.contains(key))
//	{
//		qDebug() << "You are trying to access a non existent config member" << key;
//		return sink;
//	}
//
//    real_t* ptr = (real_t*)((uintptr_t)this+memberOffsets[key]);
//    real_t& rf = *ptr;
//	return rf;
//}
