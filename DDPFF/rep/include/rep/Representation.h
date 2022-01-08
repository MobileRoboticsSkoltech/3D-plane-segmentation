#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include "globals/constants.h"

#include <memory>
#include <QPainter>

using T_ptr_t = Transform3D*;
using pointBuffer_ptr_t = pointBuffer_t*;
using colorBuffer_ptr_t = colorBuffer_t*;
using depthBuffer_ptr_t = depthBuffer_t*;

class Representation{

protected:

    /*
     * The camera transforms w.r.t to some global coordinate frame.
    */
    T_ptr_t T_ptr, T_t_ptr, invT_ptr, invT_t_ptr;

    /*
     * The buffers needed by the representations.
    */
    pointBuffer_ptr_t pBuf_ptr;
    colorBuffer_ptr_t cBuf_ptr;
    depthBuffer_ptr_t dBuf_ptr;

public:

    /*
     * Set the requisite transformations. This allows the representation to be calculated in some global fixed frame.
    */
    void setTransforms(T_ptr_t T, T_ptr_t T_t, T_ptr_t invT, T_ptr_t invT_t) {
        this->T_ptr = T; this->T_t_ptr = T_t; this->invT_ptr = invT; this->invT_t_ptr = invT_t;
    }

    /*
     * Set the requisite buffers. These buffers contain the raw data on which the represenation is based.
    */
    void setBuffers(pointBuffer_ptr_t pBuf_ptr, colorBuffer_ptr_t cBuf_ptr, depthBuffer_ptr_t dBuf_ptr) {
        this->pBuf_ptr = pBuf_ptr; this->cBuf_ptr = cBuf_ptr; this->dBuf_ptr = dBuf_ptr;
    }

    /*
     * Initialize the representation.
    */
    virtual void init() = 0;

    /*
     * Clear internal data structures.
    */
    virtual void clear() = 0;

    /*
     * Compute the representation for the current frame.
    */
    virtual void compute() = 0;

    virtual ~Representation() {}

    virtual void draw() const = 0;

    virtual void draw(QPainter* painter) const = 0;
};

#endif // REPRESENTATION_H
