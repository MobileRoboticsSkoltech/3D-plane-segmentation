#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <list>
#include <QColor>
#include <memory>
#include <unordered_map>

#include "globals/constants.h"
#include "utils/ColorUtil.h"
#include "utils/plane.h"

using colormap_t = std::unordered_map<size_t, PlanePointNormal>;

// Ray-tracing namespace
namespace rt {

/*
 * An abstract base for shape primitives.
 * It is assumed to be a body with perfect reflectance.
*/
class Primitive
{

protected:

    /* The transformation of this shape. */
    Transform3D transformation_;

    /* The scale factor of this shape. */
    real_t scale_;

public:

    /* A default primitive with center at the origin and scale equals unity. */
    Primitive() : Primitive(Transform3D::Identity(), 1) {};

    Primitive(const real_t& s) : Primitive(Transform3D::Identity(), s) {};

    Primitive(const Transform3D& t, const real_t s) : transformation_(t), scale_(s) {};
    
    virtual ~Primitive() {};

    /* Set the transformation for this primitive. */
    virtual void applyTransform(const Transform3D& t) = 0;

    /* OpenGL draw instructions. */
    virtual void draw() const = 0;

    /*
     * Intersection of this primitive with the given ray.
     * Note that `rayOrigin` and indeed the primitive parameters
     * are expressed in world frame. Thus the intersection is
     * also calculated in world frame.
    */
    virtual void intersect(const Vec3& rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const = 0;

    /* Generate the plane IDs and colors for unique identification. */
    virtual void generateColorsAndPlaneIds(size_t& planeId, RandomColorGenerator& rcg) = 0;

    virtual colormap_t getColorPlaneMap() = 0;
};


/* A box with arbitrary dimensions and location. */
class Box : public Primitive {

private:

    /* The length of the sides of the cuboid. */
    real_t a_, b_, c_;

    // corners
    // 1: top left front
    // 2: down left front
    // 3: top right front
    // 4: down right front
    // 5: top right back
    // 6: down right back
    // 7: top left back
    // 8: down left back
    // consider trb and blf as the bounds
    Vec3 tlf_, dlf_, trf_, drf_, trb_, drb_, tlb_, dlb_;

    // axis-aligned normals
    Vec3 nDown_, nTop_, nLeft_, nRight_, nFront_, nBack_;

    // Centroid of each face
    Vec3 pDown_, pTop_, pLeft_, pRight_, pFront_, pBack_;

    // Colors for all the six planes
    QColor cDown_, cTop_, cLeft_, cRight_, cFront_, cBack_;

    /*
     * Store the inverse of the transformation applied to the box.
     * This is useful to transform the rays when calculating the
     * intersections.
    */
    Transform3D invTransformation_;

    /* Plane ids for the six planes of the box. */
    size_t planeIdDown_, planeIdTop_, planeIdLeft_, planeIdRight_, planeIdFront_, planeIdBack_;

public:

    /* A default cuboid with center at the origin, scale = 1 and side-length of 1 units. */
    Box();

    Box(const Transform3D& t, const real_t s, const real_t a_, const real_t b_, const real_t c_);

    Box(const real_t s, const real_t a_, const real_t b_, const real_t c_);

    Box(const real_t a_, const real_t b_, const real_t c_);
    
    ~Box(){}

    void draw()  const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& normal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};


/*
 * A sphere.
*/
class Sphere : public Primitive {

private:

    /* The radius of the sphere and its square. */
    real_t r_, r_sqr_;

    /* The center of the sphere. */
    Vec3 center_;

    /* The plane id for the sphere is non-existent. Set a sentinel value. */
    size_t planeId_;

    QColor color_;

public:

    /* A default sphere with center at the origin, scale = 1 and radius of 1 units. */
    Sphere();

    Sphere(const Transform3D& transformation_, const real_t scale_, const real_t radius);

    Sphere(const real_t scale_, const real_t radius);

    Sphere(const real_t radius);
    
    ~Sphere(){}

    void draw() const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};


/*
 * A cylinder.
*/
class Cylinder : public Primitive {

private:

    /* The radius and length of the cylinder. */
    real_t r_, l_;

    /* The plane id for the cylinder is non-existent. Set a sentinel value. */
    size_t planeId_;

    QColor color_;

public:

    /* A default cylinder with center at the origin, scale = 1, radius of 1 units and length of 1 units. */
    Cylinder() : Primitive(), r_(1), l_(1) {};

    Cylinder(const Transform3D& t, const real_t s, const real_t r, const real_t l) : Primitive(t, s), r_(r), l_(l) {};

    Cylinder(const real_t s, const real_t r, const real_t l) : Primitive(s), r_(r), l_(l) {};

    Cylinder(const real_t r, const real_t l) : Primitive(), r_(r), l_(l) {};

    void draw() const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};


/*
 * A cone.
*/
class Cone : public Primitive {

private:

    /* The radius bottom, radius top and length of the cone. */
    real_t rb_, rt_, l_;

    /* The plane id for the cone is non-existent. Set a sentinel value. */
    size_t planeId_;

    QColor color_;

public:

    /* A default cone with center at the origin, scale = 1, radius bottom of 1 units, radius top of 1 units and length of 1 units. */
    Cone() : Primitive(), rb_(1), rt_(1), l_(1) {};

    Cone(const Transform3D& t, const real_t s, const real_t rb, const real_t rt, const real_t l) : Primitive(t, s), rb_(rb), rt_(rt), l_(l) {};

    Cone(const real_t s, const real_t rb, const real_t rt, const real_t l) : Primitive(s), rb_(rb), rt_(rt), l_(l) {};

    Cone(const real_t rb, const real_t rt, const real_t l) : Primitive(), rb_(rb), rt_(rt), l_(l) {};

    void draw() const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};



/*
 * A disk. An untransformed disk lies flat on the ground.
 * The color is red.
*/
class Disk : public Primitive {

private:

    /* The radius of the disk. */
    real_t r_;

    /* Normal to the disk. Indicates orientation. */
    Vec3 normal_;

    /* Center of the disk. */
    Vec3 center_;

    /* The plane id for the flat disk. */
    size_t planeId_;

    QColor color_;

public:

    /* A default disk with center at the origin, scale = 1 and radius of 1 units. */
    Disk();

    Disk(const Transform3D& t, const real_t s, const real_t r_);

    Disk(const real_t s, const real_t r_);

    Disk(const real_t r_);

    void draw() const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};


/*
 * A polygon. The vertices are ordered in an anti-clockwise fashion
 * when adding. This helps avoid overlappling sides when drawing.
*/
class Polygon : public Primitive {

private:

    /* The vertices of the polygon. */
    std::vector<Vec2> vertices_;

    /* The polygon is a plane represented in the point-normal form. */
    Vec3 center_, normal_;

    /* Inverse of the transformation. */
    Transform3D invTransformation_;

    /* A flag to check if the polygon is already tranformed. */
    bool transformed_;

    /* The plane id for the flat polygon. */
    size_t planeId_;

    QColor color_;

public:

    Polygon();

    Polygon(const real_t s);

    Polygon(const Transform3D& t, const real_t s);

    void draw() const override;

    void intersect(const Vec3 &rayOrigin, const Vec3& rayDirection, Vec3& intersection, Vec3& pointNormal, Pixel& color_, uint16_t& depth, size_t& planeId) const override;

    void applyTransform(const Transform3D& t) override;

    /* Overloaded. */
    Polygon& addVertex(const Vec2& vertex);
    Polygon& addVertex(const real_t& x, const real_t& y);

    void generateColorsAndPlaneIds(size_t &planeId, RandomColorGenerator &rcg) override;

    colormap_t getColorPlaneMap() override;
};

}

#endif // PRIMITIVE_H
