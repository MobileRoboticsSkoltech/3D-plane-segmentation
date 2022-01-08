#ifndef LANDSCAPEWIDGET_H
#define LANDSCAPEWIDGET_H

#include <GL/glew.h>
#include "MessageQueue.h"
#include "globals/constants.h"

#include <QGLViewer/qglviewer.h>

using namespace qglviewer;

class OpenGLViewWidget: public QGLViewer
{
	Q_OBJECT

public:
	bool recording;
    bool showPointCloud;
    bool showSuperpixels;

private:
    double radius;
    bool initialized;
    MessageQueue messageQueue;
    std::vector<QColor> colorMap;

public:
    OpenGLViewWidget(QWidget* parent=0);
    ~OpenGLViewWidget();

public slots:
	void messageIn(QString m);
    void frameIndexChangedIn(int cfi);
	void reset();

    void togglePointCloud();
    void toggleAxis();
	void togglePerspective();

protected:
    void init() override;
    void draw() override;
    void mousePressEvent(QMouseEvent *qme) override;
    void paintEvent(QPaintEvent* paintEvent) override;

private:
    void drawPoints();
    void drawGTNormals();
    void drawCameraTransform();
    void drawFloor();
    void drawGTPlanes();

signals:
    void updateUI();
};

#endif // LANDSCAPEWIDGET_H
