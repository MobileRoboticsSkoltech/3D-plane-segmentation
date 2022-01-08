#ifndef OPENGLSCENEWIDGET_H
#define OPENGLSCENEWIDGET_H

#include <GL/glew.h>
#include "scenes/scene.h"

#include <QGLViewer/qglviewer.h>
#include <vector>
#include <memory>
#include <QKeyEvent>

using namespace qglviewer;

class OpenGLSceneWidget : public QGLViewer
{
    Q_OBJECT

private:

    bool initialized;
    real_t radius;

    // Simulated scene
    scenePtr_t scene;

    /* Draw the floor in the scene as a reference. */
    void drawFloor() const;

    /* Draw the camera pose within the scene. */
    void drawCameraTransform() const;

protected:

    void init() override;
    void draw() override;
    void mousePressEvent(QMouseEvent *qme) override;
    void paintEvent(QPaintEvent* paintEvent) override;

public:

    explicit OpenGLSceneWidget(QWidget *parent = nullptr);
    ~OpenGLSceneWidget();
    
    void swapScene(const uchar& sceneId);

    void keyPressEvent(QKeyEvent *event) override;

public slots:

    void toggleAxis();

signals:

    void updateUI();
};

#endif // OPENGLSCENEWIDGET_H
