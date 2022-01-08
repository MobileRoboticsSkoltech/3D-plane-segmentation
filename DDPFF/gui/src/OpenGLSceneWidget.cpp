#include <GL/glew.h>
#include "gui/OpenGLSceneWidget.h"
#include "globals/Command.h"
#include "globals/Config.h"
#include "control/State.h"

void OpenGLSceneWidget::drawFloor() const
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
}


void OpenGLSceneWidget::drawCameraTransform() const
{
    glPushMatrix();
    glMultMatrixd(scene->getCameraPose().data());
    QGLViewer::drawAxis(0.3);
    glPopMatrix();
}

void OpenGLSceneWidget::init()
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

    initialized = true;
}

void OpenGLSceneWidget::draw()
{
    scene->draw();

    if (command.showFloor) {
        drawFloor();
    }

    if (command.showCameraTransform) {
        drawCameraTransform();
    }

    // Update the camera position. Probably should find a better place to do this.
    // As of now, every update involves a matrix inversion as well.
    scene->setCameraPose(config.sceneCameraX, config.sceneCameraY, config.sceneCameraZ, config.sceneCameraRoll, config.sceneCameraPitch, config.sceneCameraYaw);
    // Update the point buffers
    scene->render(curState.pointBuffer, curState.gtNormalBuffer, curState.colorBuffer, curState.depthBuffer, curState.gtPlaneBuffer);
    curState.update();

    emit updateUI();
}

void OpenGLSceneWidget::mousePressEvent(QMouseEvent *qme)
{
    QGLViewer::mousePressEvent(qme);
}

void OpenGLSceneWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()){
        return;
    }

    bool updated = false;

    if (event->key() == Qt::Key_Up && event->modifiers() & Qt::ControlModifier) {
        config.sceneCameraPitch += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Up && event->modifiers() & Qt::ShiftModifier) {
        config.sceneCameraZ += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Up) {
        config.sceneCameraX += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Down && event->modifiers() & Qt::ControlModifier) {
        config.sceneCameraPitch -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Down && event->modifiers() & Qt::ShiftModifier) {
        config.sceneCameraZ -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Down) {
        config.sceneCameraX -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Left && event->modifiers() & Qt::ControlModifier) {
        config.sceneCameraRoll += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Left && event->modifiers() & Qt::ShiftModifier) {
        config.sceneCameraYaw += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Left) {
        config.sceneCameraY += 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Right && event->modifiers() & Qt::ControlModifier) {
        config.sceneCameraRoll -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Right && event->modifiers() & Qt::ShiftModifier) {
        config.sceneCameraYaw -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Right) {
        config.sceneCameraY -= 0.1;
        updated = true;
    } else if (event->key() == Qt::Key_Space) {
        // Signifies no-operation; simply record the current scene and camera pose.
        updated = true;
    }

    scene->setCameraPose(config.sceneCameraX, config.sceneCameraY, config.sceneCameraZ, config.sceneCameraRoll, config.sceneCameraPitch, config.sceneCameraYaw);

    curState.cameraPose.position.x() = config.sceneCameraX; curState.cameraPose.position.y() = config.sceneCameraY; curState.cameraPose.position.z() = config.sceneCameraZ;
    curState.cameraPose.orientation.x() = config.sceneCameraRoll; curState.cameraPose.orientation.y() = config.sceneCameraPitch; curState.cameraPose.orientation.z() = config.sceneCameraYaw;

    update();
    curState.bufferAppend(config.bufferSize);
}

void OpenGLSceneWidget::paintEvent(QPaintEvent *paintEvent)
{
    QGLViewer::paintEvent(paintEvent);
}

OpenGLSceneWidget::OpenGLSceneWidget(QWidget *parent) : QGLViewer(parent)
{
    radius = 6.0;
}

OpenGLSceneWidget::~OpenGLSceneWidget()
{
    if (initialized) {
        saveStateToFile();
    }
}

void OpenGLSceneWidget::toggleAxis()
{
    toggleAxisIsDrawn();
    update();
}

void OpenGLSceneWidget::swapScene(const uchar& sceneId) {
    Vec3 translation(config.sceneCameraX, config.sceneCameraY, config.sceneCameraZ);
    Vec3 rotation(config.sceneCameraRoll, config.sceneCameraPitch, config.sceneCameraYaw);
    scene = Scene::getScene(curState.unitImage, command.openGlScene, config.sceneRadius, sceneId, translation, rotation);
}
