#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include <sstream>
#include <cassert>
#include <cmath>

#include "globals/constants.h"

// Checks if v1 is left of v2
bool isLeftOf(const Vec2& v1, const Vec2& v2);

// Checks if v1 is right of v2
bool isRightOf(const Vec2& v1, const Vec2& v2);

template<typename T, int R, int C>
bool notValid(const Eigen::Matrix<T,R,C> &v) {
    if (v.isZero(EPSILON)) {
        return true;
    }

    return false;
}

template<typename T, int R, int C>
bool operator<(const Eigen::Matrix<T,R,C> &v1, const Eigen::Matrix<T,R,C> &v2) {
    const T * const p1 = v1.data();
    const T * const p2 = v2.data();
    for (int i = 0; i < R * C; i++) {
        if (p1[i] > p2[i]) {
            return false;
        }
    }
    return true;
}

template<typename T, int R, int C>
bool operator>(const Eigen::Matrix<T,R,C> &v1, const Eigen::Matrix<T,R,C> &v2) {
    const T * const p1 = v1.data();
    const T * const p2 = v2.data();
    for (int i = 0; i < R * C; i++) {
        if (p1[i] < p2[i]) {
            return false;
        }
    }
    return true;
}

template <typename Derived>
bool operator <(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
{
    assert (v1.rows() == v2.rows() && v1.cols() == v2.cols());
    bool out = true;
    for (int i = 0; i < v1.rows(); i ++)
        for (int j = 0; j < v1.cols(); j ++)
            out = out && (v1(i,j) < v2(i,j));
    return out;
}

template <typename Derived>
bool operator <=(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
{
    assert (v1.rows() == v2.rows() && v1.cols() == v2.cols());
    bool out = true;
    for (int i = 0; i < v1.rows(); i ++)
        for (int j = 0; j < v1.cols(); j ++)
            out = out && (v1(i,j) <= v2(i,j));
    return out;
}

template <typename Derived>
bool operator >(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
{
    assert (v1.rows() == v2.rows() && v1.cols() == v2.cols());
    bool out = true;
    for (int i = 0; i < v1.rows(); i ++)
        for (int j = 0; j < v1.cols(); j ++)
            out = out && (v1(i,j) > v2(i,j));
    return out;
}

template <typename Derived>
bool operator >=(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
{
    assert (v1.rows() == v2.rows() && v1.cols() == v2.cols());
    bool out = true;
    for (int i = 0; i < v1.rows(); i ++)
        for (int j = 0; j < v1.cols(); j ++)
            out = out && (v1(i,j) >= v2(i,j));
    return out;
}

#endif
