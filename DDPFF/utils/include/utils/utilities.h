#ifndef UTILITIES_H
#define UTILITIES_H

#include <cmath>
#include <limits>

#include "globals/constants.h"
#include "eigenutils.h"

namespace utils {

    using real_t = double;

    template <typename T>
    T clip(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }

    template <typename T>
    T mean(const T& val1, const T& val2){
        return 0.5 * (val1 + val2);
    }

    /* Check if 'val' is within the intervals defined by 'bracket1' and 'bracket2'. Boundaries inclusive. */
    template <typename T>
    bool ix(const T &val, const T &bracket1, const T &bracket2){
        assert (bracket1 <= bracket2);
        return (bracket1 <= val && val <= bracket2);
    }

    /* Check if 'val' is within the intervals defined by 'bracket1' and 'bracket2'. Include 'bracket1', exclude 'bracket2'. */
    template <typename T>
    bool lix(const T &val, const T &bracket1, const T &bracket2){
        assert (bracket1 < bracket2);
        return (bracket1 <= val && val < bracket2);
    }

    /* Check if 'val' is within the intervals defined by 'bracket1' and 'bracket2'. Exclude 'bracket1', include 'bracket2'. */
    template <typename T>
    bool rix(const T &val, const T &bracket1, const T &bracket2){
        assert (bracket1 < bracket2);
        return (bracket1 < val && val <= bracket2);
    }

    /* Check if 'val' is within the intervals defined by 'bracket1' and 'bracket2'. Boundaries exclusive. */
    template <typename T>
    bool ex(const T &val, const T &bracket1, const T &bracket2){
        assert (bracket1 < bracket2);
        return (bracket1 < val && val < bracket2);
    }

    /*
     * Check if the line segments p1->p2 and p3->p4 intersect.
     * Using the method outlined in https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
    */
    template <typename T>
    void lineIntersection(const double& x1, const double& y1, const double& x2, const double& y2,
                          const double& x3, const double& y3, const double& x4, const double& y4,
                          double &t, double& u,
                          T& intersection_x, T& intersection_y){
        const double denom = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));

        if (fabs(denom) < EPSILON) {
            // parallel lines
            intersection_x = std::numeric_limits<double>::infinity();
            intersection_y = std::numeric_limits<double>::infinity();
            return;
        }

        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        u = ((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

        intersection_x = (T) (x1 + t * (x2 - x1));
        intersection_y = (T) (y1 + t * (y2 - y1));
    }

    // Min max bound trio.
    template <typename T>
    inline const T &min(const T &a, const T &b) { return (a < b) ? a : b; }
    template <typename T>
    inline const T &max(const T &a, const T &b) { return (a < b) ? b : a; }
    template <typename T>
    inline const T &bound(const T &l, const T &val, const T &u){ return max(l, min(u, val)); }

    /*
     * Round v to p places after decimal.
    */
    inline real_t round(const real_t v, const int p) {
        const int multiplier = std::pow(10, p);
        return std::round(v * multiplier) / multiplier;
    }

    /*
     * Round v to p places after decimal.
    */
    inline real_t trunc(const real_t v, const int p) {
        const int multiplier = std::pow(10, p);
        return std::trunc(v * multiplier) / multiplier;
    }
}

#endif // UTILITIES_H
