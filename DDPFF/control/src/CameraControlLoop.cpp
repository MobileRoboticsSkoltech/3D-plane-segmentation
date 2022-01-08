#include "control/CameraControlLoop.h"
#include "control/State.h"
#include "globals/Config.h"
#include "globals/Command.h"
#include "utils/Statistics.h"
#include <QDebug>

// The main control loop is the main thread of the architecture.
// A high precision windows multimedia timer drives the main thread by periodically
// calling the step method. Even if this class is not a QThread, due to the windows
// mm timer the step method is executed in a separate thread.

CameraControlLoop::CameraControlLoop(QObject *parent) : QObject(parent)
{
	// Connect the internal timer.
    connect(&timer, SIGNAL(timeout()), this, SLOT(step()), Qt::DirectConnection); // It must be a direct connection for it to work right!
	running = false;
    lastUpdateTimestamp = 0;
    lastStartTimestamp = 0;
    connect(this, SIGNAL(timeOut(double)), &cameraControl, SLOT(timeIn(double)));
}

// Initialization cascade after construction.
void CameraControlLoop::init()
{
    cameraControl.init();
}

// Reset to an initial state.
void CameraControlLoop::reset()
{

}

void CameraControlLoop::configChangedIn()
{

}

// Starts the main control loop.
void CameraControlLoop::start()
{
	running = true;
	timer.start((int)(config.systemIterationTime*1000));
	lastStartTimestamp = stopWatch.programTime();
}

// Stops the main control loop.
void CameraControlLoop::stop()
{
	running = false;
	timer.stop();
}

// The main loop of the game. It's triggered by the timer.
void CameraControlLoop::step()
{
    // This is a mutex against the gui draw() to avoid thread issues.
    // Be aware that this does influence the iteration time, but not the execution time.
    QMutexLocker locker(&gMutex);

    stopWatch.start();

	// Measure how much real time passed since the last tick.
	curState.time += config.systemIterationTime;
	curState.realTime = stopWatch.time();
    curState.rcIterationTime = curState.realTime - lastUpdateTimestamp;
	lastUpdateTimestamp = curState.realTime;

    // Step the camera control.
    stopWatch.start();
    cameraControl.step();
  
	curState.frameId++;

    // Measure execution time.
    curState.rcExecutionTime = stopWatch.elapsedTime();
    curState.avgExecutionTime = (curState.avgExecutionTime * curState.frameId + curState.rcExecutionTime) / (curState.frameId + 1);

    // Buffer the state into history.
    curState.bufferAppend(config.bufferSize);

    // Buffer also into a file if requested.
    if(command.bufferToFile)
        curState.bufferToFile();
}

// Executes a camera control step without buffering and measuring time.
// This is to recompute things for a loaded state.
void CameraControlLoop::smallStep(int frameIndex)
{
    stopWatch.start();

    cameraControl.step();

    emit timeOut(stopWatch.elapsedTimeMs());

    curState.rcExecutionTime = stopWatch.elapsedTime();
    curState.avgExecutionTime = (curState.avgExecutionTime * curState.frameId + curState.rcExecutionTime) / (curState.frameId + 1);

    curState.bufferOverwrite(frameIndex);
}

