#ifndef COMMAND_H_
#define COMMAND_H_

#include <QList>

// The global command object contains user input from the GUI.
struct Command
{
    int cameraId;
    bool bufferToFile;
    uchar cameraTransformMethod;
    uchar scene;
    bool showCameraTransform;
    bool showFloor;
    bool normals;
    bool showDDPFF;
    bool showMissingDepth;
    bool showViz;
    bool showGTPlanes;
    bool printIoUs;

    /* Flood fill*/
    bool showStartPoints;
    bool showPlaneNormals;
    bool showText;
    bool showUnmergedPlanes;
    bool showMergedPlanes;
    bool showGaps;
    
    /* Artificial scene. */
    bool recordScene;
    bool simulatedScene;
    bool openGlScene;
    bool adjustScene;

    bool experimentalFeatureOn;

	Command();
};

extern Command command;

#endif /* COMMAND_H_ */
