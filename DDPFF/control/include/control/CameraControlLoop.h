#ifndef CAMERACONTROLLOOP_H_
#define CAMERACONTROLLOOP_H_

#include "utils/StopWatch.h"
#include "utils/Timer.h"
#include "CameraControl.h"

class CameraControlLoop : public QObject
{
    Q_OBJECT

    bool running;
    StopWatch stopWatch; // for precise performance measuring
    Timer timer; // drives the camera control thread
    double lastUpdateTimestamp;
    double lastStartTimestamp;

    CameraControl cameraControl;

public:
    CameraControlLoop(QObject *parent = 0);
    ~CameraControlLoop(){}

    void init();
    bool isRunning();

public slots:
    void start();
    void stop();
    void step();
    void smallStep(int frameIndex);
    void reset();
    void configChangedIn();

signals:
    void messageOut(QString);
    void timeOut(double timeMs);
};

#endif
