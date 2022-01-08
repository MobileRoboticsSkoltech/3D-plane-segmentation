#ifndef CAMERAVIEWWIDGET_H_
#define CAMERAVIEWWIDGET_H_

#include <GL/glew.h>
#include <QWidget>
#include <QImage>
#include <QScrollArea>

#include "globals/constants.h"

class CameraViewWidget : public QWidget
{
    Q_OBJECT

private:

    QImage colorImage_;

    real_t opacity_, scale_;

    QPoint lastClickPos_, delta_;

    void drawDepthImage(QPainter *painter);

    void drawMissingDepth(QPainter *painter);

    void drawGroundTruthPlanes(QPainter *painter);

    std::vector<QColor> colorMap;

public:
    bool showDepthImage;

public:
    CameraViewWidget(QWidget *parent = 0);
    ~CameraViewWidget(){};

public slots:
    void init();
    void frameIndexChangedIn(int cfi);

protected:
    void paintEvent(QPaintEvent* paintEvent) override;
    void mousePressEvent(QMouseEvent *qme) override;
    void mouseMoveEvent(QMouseEvent *qme) override;
    void mouseDoubleClickEvent(QMouseEvent *qme) override;
    void wheelEvent(QWheelEvent *qwe) override;

signals:
    void updateUI();
};

#endif // CAMERAVIEWWIDGET_H_
