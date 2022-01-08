#ifndef SCENERENDERER_H
#define SCENERENDERER_H

#include <QGLViewer/qglviewer.h>

using namespace qglviewer;

class SceneRenderer: private QGLViewer
{
        Q_OBJECT

public:
    SceneRenderer(QWidget* parent=0);
    ~SceneRenderer();

    void init();

protected:
    void draw();

private:
    void grab() const;
    void drawScene() const;

};

#endif
