#include "utils/eigenutils.h"

// Checks if v1 is left of v2
bool isLeftOf(const Vec2& v1, const Vec2& v2) {
    return v1.x() * v2.y() - v1.y() * v2.x() < 0;
}

// Checks if v1 is right of v2
bool isRightOf(const Vec2& v1, const Vec2& v2) {
    return v1.x() * v2.y() - v1.y() * v2.x() > 0;
}

