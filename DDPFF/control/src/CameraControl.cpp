#include "control/CameraControl.h"
#include "control/State.h"
#include "globals/Command.h"
#include "globals/Config.h"
#include "globals/constants.h"

#include <assert.h>
#include <QDateTime>

CameraControl::CameraControl(QObject *parent) :
    QObject(parent)
{
}

void CameraControl::init()
{
    QMutexLocker locker(&gMutex);
    ddpff.init();
}

void CameraControl::step()
{
    computeCameraTransform();
    computeDDPFF();
    computeIoUs();
}


void CameraControl::timeIn(double timeMs)
{
    Q_UNUSED(timeMs);
}

// Computes the camera transform M using different methods that we investigate.
void CameraControl::computeCameraTransform()
{
    if (command.cameraTransformMethod == 1) // M = GT
    {
        curState.cameraTransform = curState.gtTransform;
    } else // M = I
    {
        curState.cameraTransform = Transform3D::Identity();
    }

    curState.cameraTransformT = curState.cameraTransform.matrix().transpose();
    curState.invCameraTransform = curState.cameraTransform.inverse();
    curState.invCameraTransformT = curState.cameraTransform.linear().inverse().transpose();
}

void CameraControl::computeDDPFF()
{
    if (!command.showDDPFF) {
        return;
    }
    ddpff.setBuffers(&curState.pointBuffer, &curState.colorBuffer, &curState.depthBuffer);
    ddpff.setTransforms(&curState.cameraTransform, &curState.cameraTransformT, &curState.invCameraTransform, &curState.invCameraTransformT);
    ddpff.compute();
    curState.currentRepresentation = &ddpff;
}

void CameraControl::computeIoUs()
{
    if (!command.printIoUs) {
        return;
    }

    PlanarRepresentation *pRep = dynamic_cast<PlanarRepresentation*>(curState.currentRepresentation);
    if (!pRep) {
        qWarning() << "No planar representation selected for IoU calculation.";
        return;
    }
    const auto &planes = pRep->getPlanes();

    // First collect GT plane inliers
    std::vector<std::set<size_t>> gtPlaneInliers;
    gtPlaneInliers.resize(256);
    for (size_t i = 0; i < curState.gtPlaneBuffer.size(); i++) {
        const auto label = curState.gtPlaneBuffer[i];
        if (label > 255) {
            continue;
        }
        gtPlaneInliers[label].insert(i);
    }

    std::vector<size_t> I, U;
    std::vector<real_t> ious;
    ious.resize(256, 0.0);

    // Planes IDs 0 and 1 are background
    for (size_t i = 1; i < gtPlaneInliers.size(); i++) {
        if (gtPlaneInliers[i].empty()) {
            continue;
        }

        auto &gtPlane = gtPlaneInliers[i];
        for (auto &plane: planes) {
            // Intersection
            I.clear();
            I.resize(std::max(plane.inliers.size(), gtPlane.size()));
            std::vector<size_t>::iterator it_I = std::set_intersection(gtPlane.begin(), gtPlane.end(), plane.inliers.begin(), plane.inliers.end(), I.begin());
            I.resize(it_I - I.begin());

            // Union
            U.clear();
            U.resize(plane.inliers.size() + gtPlaneInliers[i].size());
            std::vector<size_t>::iterator it_U = std::set_union(gtPlane.begin(), gtPlane.end(), plane.inliers.begin(), plane.inliers.end(), U.begin());
            U.resize(it_U - U.begin());

            real_t iou = I.size() / (1.0 * U.size());

            if (iou > ious[i]) {
                ious[i] = iou;
            }
        }
    }

    real_t totalIoU = 0.0;
    size_t divisor = 0;
    // Get average IoU
    for (size_t i = 2; i < ious.size(); i++) {
        if (gtPlaneInliers[i].empty()) {
            continue;
        }

        totalIoU += ious[i];
        divisor += 1;
    }

    qDebug() << "Frame:" << curState.frameId << "Average IoU:" << (totalIoU / divisor);
}
