#ifndef EIGENUTILS_H
#define EIGENUTILS_H

#include <QDataStream>
#include <QDebug>
#include <sstream>
#include <cassert>
#include <cmath>

#include "globals/constants.h"

// Returns the smaller, always positive angle between vectors v1 and v2.
template <typename Derived>
real_t angleTo(const Eigen::MatrixBase<Derived> &v1, const Eigen::MatrixBase<Derived> &v2)
{
    return std::acos(qBound<real_t>(-1.0, v1.dot(v2) / (v1.norm() * v2.norm()), 1.0));
}

// Checks if v1 is left of v2
bool isLeftOf(const Vec2& v1, const Vec2& v2);

// Checks if v1 is right of v2
bool isRightOf(const Vec2& v1, const Vec2& v2);

template<typename T, int R, int C>
bool notValid(const Eigen::Matrix<T,R,C> &v) {
    if (v.isZero(EPSILON)) {
        return true;
    }

//    const T * const p = v.data();
//    for(int i = 0; i < R * C; i++) {
//        if (std::isnan(p[i])) {
//            return true;
//        }
//    }

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

template <typename T, int R, int M>
QDataStream &operator<<(QDataStream &out, const Eigen::Matrix<T, R, M> &m) {
    Q_STATIC_ASSERT(R != Eigen::Dynamic && M != Eigen::Dynamic);
    for (int i = 0; i < R; i ++)
        for (int j = 0; j < M; j ++)
            out << m(i, j);
    return out;
}

template <typename T, int R, int M>
QDataStream &operator>>(QDataStream &in, Eigen::Matrix<T, R, M> &m) {
    Q_STATIC_ASSERT(R != Eigen::Dynamic && M != Eigen::Dynamic);
    for (int i = 0; i < R; i ++)
        for (int j = 0; j < M; j ++)
            in >> m(i, j);
    return in;
}

template <typename T, int R, int C>
QDebug operator<<(QDebug dbg, const Eigen::Matrix<T, R, C> &o)
{
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < o.rows(); i++){
        for (int j = 0; j < o.cols(); j++){
            ss << o(i, j);
            ss << ",";
        }
    }
    ss << "] ";

    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << QString::fromStdString(ss.str());
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << QString::fromStdString(ss.str());
    }
    return dbg;
}

template <typename Derived>
QDebug operator<<(QDebug dbg, const Eigen::MatrixBase<Derived> &o)
{
    uint R = o.rows(), C = o.cols();
    std::stringstream ss;
    ss << "[";
    for (uint i = 0; i < R; i++){
        for (uint j = 0; j < C; j++){
            ss << o(i, j);
            ss << ",";
        }
    }
    ss << "] ";

    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << QString::fromStdString(ss.str());
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << QString::fromStdString(ss.str());
    }
    return dbg;
}

template <typename S, int D, int M>
QDebug operator<<(QDebug dbg, const Eigen::Transform<S, D, M> &o)
{
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < o.rows(); i++){
        for (int j = 0; j < o.cols(); j++){
            ss << o(i, j);
            ss << ",";
        }
    }
    ss << "] ";

    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << QString::fromStdString(ss.str());
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << QString::fromStdString(ss.str());
    }
    return dbg;
}

template <typename S, int D>
QDebug operator<<(QDebug dbg, const Eigen::Translation<S, D> &o)
{
    std::stringstream ss;
    ss << "[";
    ss << o.x() << "," << o.y() << "," << o.z();
    ss << "] ";

    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << QString::fromStdString(ss.str());
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << QString::fromStdString(ss.str());
    }
    return dbg;
}

template <typename S>
QDebug operator<<(QDebug dbg, const Eigen::Quaternion<S> &o)
{
    std::stringstream ss;
    ss << "[";
    ss << o.w() << "," << o.x() << "," << o.y() << "," << o.z();
    ss << "] ";

    if (dbg.autoInsertSpaces())
    {
        dbg.setAutoInsertSpaces(false);
        dbg << QString::fromStdString(ss.str());
        dbg.setAutoInsertSpaces(true);
    }
    else
    {
        dbg << QString::fromStdString(ss.str());
    }
    return dbg;
}


#endif
