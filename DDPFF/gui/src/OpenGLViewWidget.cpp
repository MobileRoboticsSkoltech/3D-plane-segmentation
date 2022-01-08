#include "gui/OpenGLViewWidget.h"
#include "globals/Config.h"
#include "globals/Command.h"
#include "utils/GLlib.h"
#include "utils/ColorUtil.h"
#include "utils/Pose6D.h"
#include "utils/StopWatch.h"
#include "globals/constants.h"
#include "control/State.h"
#include "utils/eigenutils.h"

#include <QKeyEvent>
#include <GL/glu.h>
#include <stdlib.h>

// The OpenGLWidget offers a 3D view where basically anything can be visualized.
// It's based on the QGLViewer library that offers
// great possibilities to create an OpenGL environment, move the camera in
// it with the mouse and it also comes with a library for 3D transformations.
OpenGLViewWidget::OpenGLViewWidget(QWidget *parent) :
    QGLViewer(parent)
{
    radius = 6.0;
    recording = false;
    showPointCloud = true;
    initialized = false;
    connect(&messageQueue, SIGNAL(updated()), this, SLOT(update()));
}

void OpenGLViewWidget::reset()
{

}

void OpenGLViewWidget::init()
{
    restoreStateFromFile();

    setBackgroundColor(QColor(255,255,255));
	setForegroundColor(QColor(0,0,0));
	setFont(QFont("Helvetica", 18));

    setAxisIsDrawn(false);
	setSceneRadius(radius);

	// Light setup
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    RandomColorGenerator colorGenerator(HueSpread(10), SaturationSpread(10), ValueSpread(10));
    colorMap.resize(256);
    for (unsigned_t i = 0; i < colorMap.size(); i++) {
        colorMap[i] = colorGenerator.next();
    }
    colorMap[1] = Qt::black;
    colorMap[0] = Qt::cyan;

    initialized = true;
}

OpenGLViewWidget::~OpenGLViewWidget()
{
    if (initialized)
        saveStateToFile();
}

void OpenGLViewWidget::messageIn(QString m)
{
	messageQueue.messageIn(m);
	update();
}

void OpenGLViewWidget::frameIndexChangedIn(int cfi)
{
    Q_UNUSED(cfi);
	update();
}

void OpenGLViewWidget::togglePerspective()
{

}

void OpenGLViewWidget::toggleAxis()
{
	toggleAxisIsDrawn();
	update();
}

void OpenGLViewWidget::togglePointCloud()
{
    showPointCloud = !showPointCloud;
    update();
}

void OpenGLViewWidget::draw()
{
    // Mutex against the step of robot control loop.
    QMutexLocker locker(&gMutex);

    // Show recording state.
    if (recording)
    {
        glColor3f(0.8, 0.3, 0.3);
        drawText(10, 24, "Recording...", QFont("Helvetica", 18));
    }

    // On the top: show the message queue.
    for (int i = 0; i < messageQueue.messages.size(); i++)
    {
        glColor4f(0.3, 0.3, 0.0, messageQueue.fadeFactors[i]);
        drawText(10, 52 + i*28, messageQueue.messages[i], QFont("Helvetica", 18));
    }

    // On the bottom: show the frame id and debug information.
    glColor3f(0.3, 0.3, 0.8);
    drawText(10,
             this->height() - 10, "frame: " + QString().number(curState.frameId) + "/" + QString().number(curState.size()),
             QFont("Helvetica", 14, QFont::Light)
    );

    if (command.showFloor)
        drawFloor();

    if (command.showCameraTransform)
        drawCameraTransform();

    if (showPointCloud)
        drawPoints();

    if (command.normals)
        drawGTNormals();

    if (command.showGTPlanes)
        drawGTPlanes();

    if (curState.currentRepresentation != nullptr)
    {
        curState.currentRepresentation->draw();
    }

    emit updateUI();
}

void OpenGLViewWidget::mousePressEvent(QMouseEvent *qme)
{
    QGLViewer::mousePressEvent(qme);
    update();
}

void OpenGLViewWidget::paintEvent(QPaintEvent *paintEvent)
{
    QGLViewer::paintEvent(paintEvent);
}

// Draws the floor.
void OpenGLViewWidget::drawFloor()
{
    float size = 2.0*radius;
    float stride = 1.0;
    glBegin( GL_QUADS );
    glColor3f(0.95, 0.95, 0.95);
    glVertex3f(-size, -size, 0);
    glVertex3f(-size, size, 0);
    glVertex3f(size, size, 0);
    glVertex3f(size, -size, 0);
    glEnd();

    glLineWidth(1);
    glBegin( GL_LINES );
    glColor3f(0.65, 0.65, 0.65);
    for (float i = -size; i <= size + 0.0001; i = i+stride)
    {
        glVertex3f(i, -size, 0.001);
        glVertex3f(i, size, 0.001);
        glVertex3f(-size, i, 0.001);
        glVertex3f(size, i, 0.001);
    }
    glEnd();

    emit updateUI();
}

// Draw ground truth planes from the artificial scene
void OpenGLViewWidget::drawGTPlanes()
{
//    if (!command.simulatedScene) {
//        return;
//    }

    glPushMatrix();
    glMultMatrixd(curState.cameraTransform.data());
    glPointSize(3);
    glBegin(GL_POINTS);

    for(size_t i = 0; i < NUMBER_OF_POINTS; i++) {
        const unsigned_t& planeId = curState.gtPlaneBuffer[i];
        const Vec3& point = curState.pointBuffer[i];
        const QColor& color = colorMap[planeId];
        glColor3f(color.redF(), color.greenF(), color.blueF());
        glVertex3d(point.x(), point.y(), point.z());
    }

    glEnd();
    glPopMatrix();
}


// Draws the camera transform.
void OpenGLViewWidget::drawCameraTransform()
{
    glPushMatrix();
    glMultMatrixd(curState.cameraTransform.data());
    QGLViewer::drawAxis(0.3);
    glPopMatrix();
}


// Draw the point buffer.
void OpenGLViewWidget::drawPoints()
{
    glPushMatrix();
    glMultMatrixd(curState.cameraTransform.data());
    glPointSize(3);
    glBegin(GL_POINTS);

    for (int r = 0; r < IMAGE_HEIGHT; r++)
    {
        if (r % config.sampleFactor != 0) {
            continue;
        }
        for (int c = 0; c < IMAGE_WIDTH; c++) {

            if (c % config.sampleFactor != 0) {
                continue;
            }

            const int i = r * IMAGE_WIDTH + c;

            if (notValid(curState.pointBuffer[i]))
                continue;

            const Pixel& color = curState.colorBuffer[i];
            glColor4ub(color.x(), color.y(), color.z(), 128);
            glVertex3dv(curState.pointBuffer[i].data());
        }
    }

    glEnd();
    glPopMatrix();
}

void OpenGLViewWidget::drawGTNormals()
{
    glPushMatrix();
    glMultMatrixd(curState.cameraTransform.data());

    static Vec3 from, to;

    for (int i = 0; i < NUMBER_OF_POINTS; i+= std::max<int>(1, config.normalSampleDistance_min))
    {
        if (notValid(curState.pointBuffer[i]))
            continue;

        glColor3ub(255, 0, 0); // red

        from = curState.pointBuffer[i];
        to = curState.pointBuffer[i] + 0.1 * curState.gtNormalBuffer[i].normalized();

        drawArrow(
                    qglviewer::Vec(from.x(), from.y(), from.z()),
                    qglviewer::Vec(to.x(), to.y(), to.z()),
                    0.001
        );
    }

    glPopMatrix();
}
