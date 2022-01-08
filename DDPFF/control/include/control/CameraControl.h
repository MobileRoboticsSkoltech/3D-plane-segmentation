#ifndef CAMERACONTROL_H
#define CAMERACONTROL_H
#include <QObject>

#include "rep/DDPFF.h"
#include "scenes/scene.h"

class CameraControl : public QObject
{
    Q_OBJECT

private:

    DDPFF ddpff;

    void computeCameraTransform();
    void computeDDPFF();
    void computeIoUs();


public:

    CameraControl(QObject *parent = 0);
    ~CameraControl(){}

    void init();
    void step();

signals:
    void messageOut(QString);

public slots:
    void timeIn(double timeMs);

};

#endif
