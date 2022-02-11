#ifndef REPRESENTATION_H
#define REPRESENTATION_H

#include "globals/constants.h"

#include <memory>

using T_ptr_t = Transform3D*;
using pointBuffer_ptr_t = pointBuffer_t*;
using colorBuffer_ptr_t = colorBuffer_t*;
using depthBuffer_ptr_t = depthBuffer_t*;

class Representation{

protected:

    /*
     * The buffers needed by the representations.
    */
    pointBuffer_ptr_t pBuf_ptr = nullptr;
    colorBuffer_ptr_t cBuf_ptr = nullptr;
    depthBuffer_ptr_t dBuf_ptr = nullptr;

public:
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
};

#endif // REPRESENTATION_H
