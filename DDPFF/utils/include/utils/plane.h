#ifndef PLANE_H_
#define PLANE_H_

#include "globals/constants.h"

#include <set>

/*
 * Plane represented in point-normal form.
*/
struct PlanePointNormal
{

    uint id;

    size_t count;

    Vec3 p,n;

    PlanePointNormal() {}

    std::set<size_t> inliers;

    inline void clear() {
        id = 0;
        count = 0;
        p.fill(0); n.fill(0);
        inliers.clear();
    }

};

/*
 * Plane represented by its linear equation.
*/
struct PlaneEquation
{

    uint id;

    real_t a, b, c, d;

    PlaneEquation() {}

};

#endif
