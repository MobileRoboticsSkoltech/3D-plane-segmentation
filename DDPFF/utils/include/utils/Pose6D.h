#ifndef POSE6D_H
#define POSE6D_H

#include "globals/constants.h"
#include "utils/eigenutils.h"

/*
 * A six dimensional pose object.It encapsulates
 * the position and orientation of a point.
 */
struct Pose6D{

    Pose6D(){}

    ~Pose6D(){}

    Vec3 position;
    Vec3 orientation;

    bool operator==(const Pose6D &o){
        if (position == o.position && orientation == o.orientation) {
            return true;
        }
        return false;
    }

    bool isInvalid() const{
        return notValid(position) || notValid(orientation);
    }

    bool isNull() const{
        return position.isZero() || orientation.isZero();
    }

    void clear(){
        position.fill(0);
        orientation.fill(0);
    }
};

#endif // POSE6D_H
