#include "globals/Command.h"
#include "globals/constants.h"

// The global command object contains user input from the GUI.
Command command;

Command::Command()
{
    cameraId = 0;
    cameraTransformMethod = 3;
    scene = 3;
    showCameraTransform = false;
    showFloor = false;
    bufferToFile = false;
    normals = false;
    showViz = false;
    showDDPFF = false;
    showMissingDepth = false;
    showGTPlanes = false;
    printIoUs = false;

    showStartPoints = false;
    showPlaneNormals = false;
    showText = false;
    showUnmergedPlanes = false;
    showMergedPlanes = false;
    
    simulatedScene = false;
    recordScene = false;
    openGlScene = true;
    adjustScene = false;

    experimentalFeatureOn = false;
}

