#ifndef PLANARREPRESENTATION_H
#define PLANARREPRESENTATION_H

#include "Representation.h"
#include "globals/constants.h"
#include "utils/plane.h"

#include <vector>

class PlanarRepresentation : public Representation {

public:

    virtual std::vector<unsigned_t> getLabels() = 0;

    virtual std::vector<PlanePointNormal> getPlanes() = 0;

};

#endif // PLANARREPRESENTATION_H
