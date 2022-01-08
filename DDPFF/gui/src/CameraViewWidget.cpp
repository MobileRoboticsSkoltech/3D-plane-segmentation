#include "gui/CameraViewWidget.h"
#include "globals/Config.h"
#include "globals/Command.h"
#include "globals/constants.h"
#include "utils/ColorUtil.h"
#include "utils/utilities.h"
#include "control/State.h"

#include <QtGlobal>
#include <QMouseEvent>
#include <QPainter>
#include <QMutexLocker>


void CameraViewWidget::drawDepthImage(QPainter *painter)
{
    painter->save();
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            const uint16_t& depth = curState.depthBuffer[y * IMAGE_WIDTH + x];
            if (depth > 0) {
                const QColor &c = colorUtil.getHeatMapColor(depth, 0, 3500); // 3.5m is the maximum depth as per specs
                QPen pen;
                pen.setColor(c);
                pen.setWidth(1);
                painter->setPen(pen);
                painter->drawPoint(x,y);
            }
        }
    }
    painter->restore();
}

void CameraViewWidget::drawMissingDepth(QPainter *painter)
{
    painter->save();

    QPen pen;
    pen.setColor(Qt::black);
    pen.setWidth(2);
    painter->setPen(pen);

    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            if (notValid(curState.pointBuffer[y * IMAGE_WIDTH + x])) {
                painter->drawPoint(x,y);
            }
        }
    }
    painter->restore();
}

void CameraViewWidget::drawGroundTruthPlanes(QPainter *painter)
{
    for (uint r = 0; r < IMAGE_HEIGHT; r++) {
        for (uint c = 0; c < IMAGE_WIDTH; c++) {
            const uint index = r * IMAGE_WIDTH + c;
            const QColor &color = colorMap[curState.gtPlaneBuffer[index]];
            painter->setPen(color);
            painter->drawPoint(c,r);
        }
    }
}

CameraViewWidget::CameraViewWidget(QWidget *parent)
    : QWidget(parent), opacity_(1.0), scale_(1.0), lastClickPos_(0,0), delta_(0,0), showDepthImage(false)
{
    setMinimumWidth(IMAGE_WIDTH);
    setMinimumHeight(IMAGE_HEIGHT);
}

void CameraViewWidget::init()
{
    RandomColorGenerator colorGenerator(HueSpread(10), SaturationSpread(10), ValueSpread(10));
    colorMap.resize(256);
    for (unsigned_t i = 0; i < colorMap.size(); i++) {
        colorMap[i] = colorGenerator.next();
    }
    colorMap[1] = Qt::black;
    colorMap[0] = Qt::cyan;
}

void CameraViewWidget::frameIndexChangedIn(int cfi)
{
    Q_UNUSED(cfi);
    // Construct a new QImage from the raw data buffer in the state.
    colorImage_ = QImage((uchar*)curState.colorBuffer.data(), IMAGE_WIDTH, IMAGE_HEIGHT, QImage::Format_RGB888);
    update();
}

void CameraViewWidget::paintEvent(QPaintEvent*)
{
    // Mutex against the robot control loop.
    QMutexLocker locker(&gMutex);

    QPainter painter(this);

    painter.scale(scale_, scale_);
    painter.translate(delta_);

    if (showDepthImage) {
        drawDepthImage(&painter);
    }else {
        // draw the camera color image
        painter.save();
        painter.setOpacity(opacity_);
        painter.drawImage(QPoint(), colorImage_);
        painter.restore();
    }

    if (curState.currentRepresentation != nullptr || curState.currentRepresentation != 0)
    {
        curState.currentRepresentation->draw(&painter);
    }

    if (command.showMissingDepth) {
        drawMissingDepth(&painter);
    }

    if (command.showGTPlanes) {
        drawGroundTruthPlanes(&painter);
    }
}

void CameraViewWidget::mousePressEvent(QMouseEvent *qme)
{
    if (qme->button() == Qt::LeftButton) {
        lastClickPos_ = qme->pos();
    }

    QWidget::mousePressEvent(qme);
    update();
}

void CameraViewWidget::mouseMoveEvent(QMouseEvent *qme)
{
    if (qme->buttons() & Qt::LeftButton) {
        delta_ = qme->pos() - lastClickPos_;
    }

    QWidget::mouseMoveEvent(qme);
    update();
}

void CameraViewWidget::mouseDoubleClickEvent(QMouseEvent *qme)
{
    if (qme->button() == Qt::LeftButton) {
        scale_ = 1.0;
        delta_.setX(0); delta_.setY(0);
    }

    QWidget::mouseDoubleClickEvent(qme);
    update();
}

void CameraViewWidget::wheelEvent(QWheelEvent *qwe)
{
    static const real_t opacityMultFactor = 0.1;
    static const real_t scaleMultFactor = 0.1;

    if (qwe->modifiers() == Qt::ControlModifier) {
        opacity_ += opacityMultFactor * qwe->angleDelta().y() / 120.0;
        opacity_ = utils::clip(opacity_, 0.0, 1.0);
    } else {
        scale_ += scaleMultFactor * qwe->angleDelta().y() / 120.0;
        scale_ = utils::clip(scale_, 0.25, 4.0);
    }
    update();
}
