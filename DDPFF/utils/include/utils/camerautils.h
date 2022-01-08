#ifndef CAMERAUTILS_H
#define CAMERAUTILS_H

#include "globals/constants.h"

#include <cstdlib>

namespace utils {

    /* The unit image is the image formed of the plane situated at one unit distance from the camera center. */
    pointBuffer_t initializeUnitImage();

    /* Normalize each ray of the unit image. */
    pointBuffer_t initializeNormalizedUnitImage();

    /* Construct the static camera projection matrix. */
    Transform3D makeCameraProjectionMatrix();

    // Camera's unit image
    static const pointBuffer_t unitImage = initializeUnitImage();

    // Camera's unit image but each ray is normalized
    static const pointBuffer_t normalizedUnitImage = initializeNormalizedUnitImage();

    // The camera projection matrix
    static const Transform3D cameraProjection = makeCameraProjectionMatrix();
}

#endif // CAMERAUTILS_H
