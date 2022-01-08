#include "scenes/primitive.h"
#include "utils/GLlib.h"
#include "utils/utilities.h"
#include "utils/eigenutils.h"

#include <QtGlobal>
#include <GL/glu.h>
#include <QGLViewer/qglviewer.h>

namespace rt {

Sphere::Sphere() : Sphere(Transform3D(), 1, 1) {}

Sphere::Sphere(const Transform3D &transformation, const real_t scale, const real_t radius) : Primitive(transformation, scale), r_(scale * radius)
{
    center_ = Vec3(0,0,0); // center at origin
    r_sqr_ = r_ * r_;
    color_ = Qt::blue;
    this->scale_ = scale;
}

Sphere::Sphere(const real_t scale, const real_t radius) : Sphere(Transform3D(), scale, radius) {}

Sphere::Sphere(const real_t radius) : Sphere(Transform3D(), 1, radius) {}

void Sphere::draw() const
{
    glPushMatrix();
    glMultMatrixd(transformation_.data());

    glColor3ub(color_.red(), color_.green(), color_.blue());

    GLlib::drawSphere(r_);
    
    glPopMatrix();
}

void Sphere::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &pointNormal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    intersection.fill(0);
    pointNormal.fill(0);
    color.fill(0);
    depth = 0;

    real_t t0, t1; // solutions for t if the ray intersects (can intersect twice)

    // geometric solution
    const Vec3& L = center_ - rayOrigin;
    real_t tca = L.dot(rayDirection);

    if (tca < 0) {
        return;
    }

    real_t d_sqr = L.dot(L) - tca * tca;
    if (d_sqr > r_sqr_) {
        return;
    }

    real_t thc = sqrt(r_sqr_ - d_sqr);
    t0 = tca - thc;
    t1 = tca + thc;

    if (t0 > t1) {
        std::swap(t0, t1);
    }

    if (t0 < 0) {
        t0 = t1; // if t0 is negative, use t1 instead
        if (t0 < 0) {
            // both t0 and t1 are negative
            return;
        }
    }

    intersection = rayOrigin + t0 * rayDirection;
    pointNormal = (intersection - center_);
    pointNormal.normalize();
    depth = 1000 * t0;
    color << color_.red(), color_.green(), color_.blue();
    planeId = this->planeId_;
}

void Sphere::applyTransform(const Transform3D &t)
{
    transformation_ = t;
    Vec3 oldCenter = center_;
    center_ = transformation_ * center_;
}

void Sphere::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    Q_UNUSED(planeId);
    planeId_ = MAXVAL(size_t);
    color_ = rcg.next();
}

colormap_t Sphere::getColorPlaneMap()
{
    return colormap_t();
}

void Cylinder::draw() const
{
    glPushMatrix();
    glMultMatrixd(transformation_.data());

    glColor3ub(color_.red(), color_.green(), color_.blue());

    GLlib::drawCylinder(r_, l_);

    glPopMatrix();
}

void Cylinder::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &pointNormal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    Q_UNUSED(rayOrigin);
    Q_UNUSED(rayDirection);
    Q_UNUSED(intersection);
    Q_UNUSED(pointNormal);
    Q_UNUSED(color);
    Q_UNUSED(depth);
    Q_UNUSED(planeId);
}

void Cylinder::applyTransform(const Transform3D &t)
{
    Q_UNUSED(t);
}

void Cylinder::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    Q_UNUSED(planeId);
    planeId_ = MAXVAL(size_t);
    color_ = rcg.next();
}

colormap_t Cylinder::getColorPlaneMap()
{
    return colormap_t();
}

void Cone::draw() const
{
    glPushMatrix();
    glMultMatrixd(transformation_.data());

    glColor3ub(color_.red(), color_.green(), color_.blue());

    GLlib::drawCone(rb_, rt_, l_);

    glPopMatrix();
}

void Cone::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &pointNormal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    Q_UNUSED(rayOrigin);
    Q_UNUSED(rayDirection);
    Q_UNUSED(intersection);
    Q_UNUSED(pointNormal);
    Q_UNUSED(color);
    Q_UNUSED(depth);
    Q_UNUSED(planeId);
}

void Cone::applyTransform(const Transform3D &t)
{
    Q_UNUSED(t);
}

void Cone::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    Q_UNUSED(planeId);
    planeId_ = MAXVAL(size_t);
    color_ = rcg.next();
}

colormap_t Cone::getColorPlaneMap()
{
    return colormap_t();
}

Disk::Disk() : Primitive(), r_(1)
{

}

Disk::Disk(const Transform3D &t, const real_t s, const real_t r) : Primitive(t, s), r_(r)
{
    this->transformation_ = t;
    this->scale_ = s;
    this->r_ = s * r;
    normal_ << 0,0,1;
    center_ << 0,0,0;
    color_ = Qt::red;
}

Disk::Disk(const real_t s, const real_t r) : Disk(Transform3D(), s, r) {}

Disk::Disk(const real_t r) : Disk(Transform3D(), 1, r) {}

void Disk::draw() const
{
    glColor3ub(color_.red(), color_.green(), color_.blue());

    glPushMatrix();
    glMultMatrixd(transformation_.data());

    GLlib::drawFilledCircle(r_);

    glPopMatrix();

    QGLViewer::drawArrow(qglviewer::Vec(center_), qglviewer::Vec(center_ + (0.5 * r_ * normal_.normalized())), 0.01);
}


void Disk::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &pointNormal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    intersection.fill(0);
    pointNormal.fill(0);
    color.fill(0);
    depth = 0;

    // assuming vectors are all normalized
    real_t denom = normal_.dot(rayDirection);
    if (fabs(denom) > EPSILON) {
        const Vec3& p0l0 = center_ - rayOrigin;
        real_t t = (p0l0.dot(normal_)) / denom;
        if (t < 0) {
            return;
        }
        intersection = rayOrigin + t * rayDirection;
        const Vec3& diff = intersection - center_;
        if (diff.norm() > r_) {
            // intersection doesn't lie within the disk's boundaries
            intersection.fill(0);
            return;
        }
        depth = 1000 * t;
        pointNormal = normal_;
        color << color_.red(), color_.green(), color_.blue();
        planeId = this->planeId_;
    }
}

void Disk::applyTransform(const Transform3D &t)
{
    this->transformation_ = t;

    Vec3 oldCenter(center_);
    Vec3 oldNormal(normal_);

    center_ = transformation_ * center_;
    normal_ = (transformation_.linear().inverse().transpose() * normal_).normalized();
}

void Disk::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    planeId_ = planeId;
    planeId++;
    color_ = rcg.next();
}

colormap_t Disk::getColorPlaneMap()
{
    colormap_t cMap;

    PlanePointNormal plane;
    plane.id = planeId_; plane.n = normal_; plane.p = center_;

    cMap[utils::rgbIndex(color_)] = plane;

    return cMap;
}

Polygon::Polygon() : Polygon(Transform3D(), 1) {
}

Polygon::Polygon(const real_t s) : Polygon(Transform3D(), s) {}

Polygon::Polygon(const Transform3D &t, const real_t s) :
    Primitive(t, s),
    center_(0,0,0),
    normal_(0,0,1),
    transformed_(false)
{
    color_ = Qt::cyan;
}

void Polygon::draw() const
{
    glPushMatrix();
    glMultMatrixd(transformation_.data());

    glColor3ub(color_.red(), color_.green(), color_.blue());

    glLineWidth(6);
    // this routine can draw only convex polygons in 3D space
    glBegin(GL_POLYGON);
    static Vec3 pt;
    for (const Vec2& v: vertices_) {
        pt.x() = v.x(); pt.y() = v.y(); pt.z() = 0;
        glVertex3dv(pt.data());
    }
    glEnd();

    glPopMatrix();

}


void Polygon::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &pointNormal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    intersection.fill(0);
    pointNormal.fill(0);
    color.fill(0);
    depth = 0;

    // assuming vectors are all normalized
    real_t denom = normal_.dot(rayDirection);
    if (fabs(denom) < EPSILON) {
        return;
    }

    const Vec3& p0l0 = center_ - rayOrigin;
    real_t t = (p0l0.dot(normal_)) / denom;
    if (t < 0) {
        return;
    }
    intersection = rayOrigin + t * rayDirection;

    // since we construct the polygon by transforming a 2D polygon on the xy plane,
    // we should be guranteed that the inverse transformation should make all points
    // on the polygonal plane lie on the xy plane.
    thread_local Vec3 intersectionXY;
    intersectionXY.fill(0);

    intersectionXY = invTransformation_ * intersection;

    if (fabs(intersectionXY.z()) > EPSILON) {
        intersection.fill(0);
        return;
    }

    uint counter = 0;

    thread_local Vec2 v1, v2;
    v1.fill(0); v2.fill(0);

    thread_local real_t u1, u2;
    thread_local real_t x, y;

    // use the even-odd algorithm to determine if the intersection point
    // is contained within the polygon.
    for (std::vector<Vec2>::const_iterator it = vertices_.begin(); it != vertices_.end(); ++it) {
        v1 = *it;

        if (std::next(it) == vertices_.end()) {
            v2 = *vertices_.begin();
        }else {
            v2 = *std::next(it);
        }

        utils::lineIntersection(intersectionXY.x(), intersectionXY.y(), intersectionXY.x(), std::numeric_limits<real_t>::max(),
                                v1.x(), v1.y(), v2.x(), v2.y(),
                                u1, u2, x, y);

        if (fabs(u2) <= 1.0) {
            counter++;
        }

    }

    if (!(counter % 2)) {
        // even : point is outside
        intersection.fill(0);
        return;
    }

    pointNormal = normal_;
    pointNormal.normalize();
    color << color_.red(), color_.green(), color_.blue();
    depth = 1000 * t;
    planeId = this->planeId_;
}

void Polygon::applyTransform(const Transform3D &t)
{
    this->transformation_ = t;

    center_ = transformation_ * center_;

    normal_ = (transformation_.linear() * normal_).normalized();

    invTransformation_ = transformation_.inverse();

    transformed_ = true;
}

Polygon &Polygon::addVertex(const Vec2 &vertex)
{
    if (transformed_) {
        // if the polygon is already transformed,
        // don't add new vertices to it.
        qWarning() << "Cannot add new vertex: Polygon already transformed.";
        return *this;
    }

    vertices_.emplace_back(scale_ * vertex);

    // update the centroid
    Vec3 accumulator;
    for (std::vector<Vec2>::iterator it = vertices_.begin(); it != vertices_.end(); ++it) {
        const Vec2& vertex = *it;
        accumulator.x() += vertex.x(); accumulator.y() += vertex.y();
    }

    // An untransformed polygon lies flat on the xy plane.
    // So here we don't need to consider the z-component
    // while calculating the centroid.
    center_ = accumulator / vertices_.size();

    return *this;
}

Polygon &Polygon::addVertex(const real_t &x, const real_t &y)
{
    addVertex(Vec2(x,y));
    return *this;
}

void Polygon::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    planeId_ = planeId;
    planeId++;
    color_ = rcg.next();
}

colormap_t Polygon::getColorPlaneMap()
{
    colormap_t cMap;
    PlanePointNormal plane;

    plane.id = planeId_;
    plane.p = center_; plane.n = normal_;

    cMap[utils::rgbIndex(color_)] = plane;

    return cMap;
}

Box::Box() : Box(Transform3D::Identity(), 1, 1, 1, 1) {}

Box::Box(const Transform3D &t, const real_t s, const real_t a, const real_t b, const real_t c) :
    Primitive(t, s), a_(a), b_(b), c_(c),
    nDown_(0,0,-1), nTop_(0,0,1), nLeft_(0,1,0), nRight_(0,-1,0), nFront_(-1,0,0), nBack_(1,0,0)
{
    this->scale_ = s;

    // scale
    this->a_ = scale_ * a;
    this->b_ = scale_ * b;
    this->c_ = scale_ * c;

    // calculate half-side lengths
    real_t half_a = 0.5 * this->a_;
    real_t half_b = 0.5 * this->b_;
    real_t half_c = 0.5 * this->c_;

    // record the transform
    transformation_ = t;
    invTransformation_ = transformation_.inverse();

    // calculate corners
    tlf_ << -half_a, half_b, half_c;
    dlf_ << -half_a, half_b, -half_c;
    trf_ << -half_a, -half_b, half_c;
    drf_ << -half_a, -half_b, -half_c;
    tlb_ << half_a, half_b, half_c;
    dlb_ << half_a, half_b, -half_c;
    trb_ << half_a, -half_b, half_c;
    drb_ << half_a, -half_b, -half_c;

    // calculate face centroids
    pTop_ = (tlf_ + trf_ + tlb_ + trb_) / 4.0;
    pDown_ = (dlf_ + drf_ + dlb_ + drb_) / 4.0;
    pLeft_ = (tlf_ + tlb_ + dlb_ + dlf_) / 4.0;
    pRight_ = (trf_ + trb_ + drb_ + drf_) / 4.0;
    pFront_ = (tlf_ + trf_ + drf_ + dlf_) / 4.0;
    pBack_ = (tlb_ + trb_ + drb_ + dlb_) / 4.0;

    // assign a default green color to the cube
    cDown_ = Qt::green; cTop_ = Qt::green; cLeft_ = Qt::green; cRight_ = Qt::green; cFront_ = Qt::green; cBack_ = Qt::green;
}

Box::Box(const real_t s, const real_t a, const real_t b, const real_t c) : Box(Transform3D::Identity(), s, a, b, c) {}

Box::Box(const real_t a, const real_t b, const real_t c) : Box(Transform3D::Identity(), 1, a, b, c) {}

void Box::draw() const
{
    glPushMatrix();
    glMultMatrixd(transformation_.data());

    GLlib::drawBox(
        tlf_, dlf_,
        trf_, drf_,
        trb_, drb_,
        tlb_, dlb_,
        cTop_, cDown_,
        cLeft_, cRight_,
        cFront_, cBack_
    );

    glPopMatrix();
}

void Box::intersect(const Vec3 &rayOrigin, const Vec3 &rayDirection, Vec3 &intersection, Vec3 &normal, Pixel &color, uint16_t &depth, size_t &planeId) const
{
    real_t tmin, tmax, tymin, tymax, tzmin, tzmax;
    static Vec3 _rayOrigin, _rayDirection, _invRayDirection, _intersection, minNorm, maxNorm, tmp;
    size_t minPlaneId, maxPlaneId;

    normal.fill(0);
    intersection.fill(0);
    color.fill(0);
    depth = 0;

    // transform the ray origin and direction to the axis-aligned frame
    _rayOrigin = invTransformation_ * rayOrigin;
    _rayDirection = invTransformation_.linear() * rayDirection;
    _invRayDirection = _rayDirection.cwiseInverse();

    const Vec3& minBound = dlf_;
    const Vec3& maxBound = trb_;

    minNorm = nBack_; minPlaneId = planeIdBack_;
    maxNorm = nFront_; maxPlaneId = planeIdFront_;

    tmin = ((_invRayDirection.x() < 0 ? maxBound : minBound).x() - _rayOrigin.x()) * _invRayDirection.x();
    tmax = ((_invRayDirection.x() < 0 ? minBound : maxBound).x() - _rayOrigin.x()) * _invRayDirection.x();
    tymin = ((_invRayDirection.y() < 0 ? maxBound : minBound).y() - _rayOrigin.y()) * _invRayDirection.y();
    tymax = ((_invRayDirection.y() < 0 ? minBound : maxBound).y() - _rayOrigin.y()) * _invRayDirection.y();

    if ((tmin > tymax) || (tymin > tmax)){
        return;
    }

    if (tymin > tmin){
        tmin = tymin;
        minNorm = nLeft_; minPlaneId = planeIdLeft_;
    }
    if (tymax < tmax){
        tmax = tymax;
        maxNorm = nRight_; maxPlaneId = planeIdRight_;
    }

    tzmin = ((_invRayDirection.z() < 0 ? maxBound : minBound).z() - _rayOrigin.z()) * _invRayDirection.z();
    tzmax = ((_invRayDirection.z() < 0 ? minBound : maxBound).z() - _rayOrigin.z()) * _invRayDirection.z();

    if ((tmin > tzmax) || (tzmin > tmax)){
        return;
    }

    if (tzmin > tmin){
        tmin = tzmin;
        minNorm = nTop_; minPlaneId = planeIdTop_;
    }
    if (tzmax < tmax){
        tmax = tzmax;
        maxNorm = nDown_; maxPlaneId = planeIdDown_;
    }

    if (tmin < 0 && tmax < 0) {
        return;
    }

    if (tmax < tmin) {
        std::swap(tmin, tmax);
        std::swap(minPlaneId, maxPlaneId);
        std::swap(minNorm, maxNorm);
    }

    if (tmin > 0) {
        _intersection = _rayOrigin + tmin * _rayDirection;
        depth = 1000 * tmin;
        normal = minNorm;
        planeId = minPlaneId;
    }else {
        _intersection = _rayOrigin + tmax * _rayDirection;
        depth = 1000 * tmax;
        normal = maxNorm;
        planeId = maxPlaneId;
    }

    // transform the intersection point to the original frame.
    intersection = transformation_ * _intersection;
    normal = transformation_.linear() * normal;
    normal.normalize();
}

void Box::applyTransform(const Transform3D &t)
{
    // save the transformation
    transformation_ = t;
    invTransformation_ = transformation_.inverse();
}

void Box::generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg)
{
    planeIdDown_ = planeId++; cDown_ = rcg.next();
    planeIdTop_ = planeId++; cTop_  = rcg.next();
    planeIdLeft_ = planeId++; cLeft_ = rcg.next();
    planeIdRight_ = planeId++; cRight_ = rcg.next();
    planeIdFront_ = planeId++; cFront_ = rcg.next();
    planeIdBack_ = planeId++; cBack_ = rcg.next();
}

colormap_t Box::getColorPlaneMap()
{
    colormap_t cMap;

    const auto& normalTransform = transformation_.linear();

    // Top
    PlanePointNormal planeTop;
    planeTop.id = planeIdTop_; planeTop.p = transformation_ * pTop_; planeTop.n = (normalTransform * nTop_).normalized();
    cMap[utils::rgbIndex(cTop_)] = planeTop;

    // Down
    PlanePointNormal planeDown;
    planeDown.id = planeIdDown_; planeDown.p = transformation_ * pDown_; planeDown.n = (normalTransform * nDown_).normalized();
    cMap[utils::rgbIndex(cDown_)] = planeDown;

    // Left
    PlanePointNormal planeLeft;
    planeLeft.id = planeIdLeft_; planeLeft.p = transformation_ * pLeft_; planeLeft.n = (normalTransform * nLeft_).normalized();
    cMap[utils::rgbIndex(cLeft_)] = planeLeft;

    // Right
    PlanePointNormal planeRight;
    planeRight.id = planeIdTop_; planeRight.p = transformation_ * pRight_; planeRight.n = (normalTransform * nRight_).normalized();
    cMap[utils::rgbIndex(cRight_)] = planeRight;

    // Front
    PlanePointNormal planeFront;
    planeFront.id = planeIdFront_; planeFront.p = transformation_ * pFront_; planeFront.n = (normalTransform * nFront_).normalized();
    cMap[utils::rgbIndex(cFront_)] = planeFront;

    // Back
    PlanePointNormal planeBack;
    planeBack.id = planeIdBack_; planeBack.p = transformation_ * pBack_; planeBack.n = (normalTransform * nBack_).normalized();
    cMap[utils::rgbIndex(cBack_)] = planeBack;

    return cMap;
}

}
