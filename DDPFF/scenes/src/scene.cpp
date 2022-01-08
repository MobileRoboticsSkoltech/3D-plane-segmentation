#include <GL/glew.h>
#include "scenes/scene.h"
#include "utils/eigenutils.h"
#include "utils/utilities.h"
#include "utils/GLlib.h"

const Vec3 Scene::origin_ = Vec3(0,0,0);
const QColor Scene::noPlane = Qt::black;


/* A helper method to construct an Eigen affine transformation matrix from translation and rotation magnitudes. */
void populateTransform(const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y, Transform3D& t) {
    t = Eigen::Translation<real_t, 3>(x,y,z);
    t.rotate(Eigen::AngleAxis<real_t>(R, Vec3::UnitX()));
    t.rotate(Eigen::AngleAxis<real_t>(P, Vec3::UnitY()));
    t.rotate(Eigen::AngleAxis<real_t>(Y, Vec3::UnitZ()));
}

/* Get the floor represented as a polygon. */
rt::Primitive* getFloor()
{
    const real_t size = 6;
    Transform3D t_floor;
    rt::Polygon* p = new rt::Polygon();
    p->addVertex(size, size).addVertex(-size, size).addVertex(-size, -size).addVertex(size, -size);
    p->applyTransform(t_floor);
    return p;
}

Transform3D getFrustum(const real_t far, const pointBuffer_t& unitImage) {
    real_t near = CAMERA_PLANE_DISTANCE;
    real_t top = unitImage[0].z();
    real_t bottom = -top;
    real_t right = unitImage[0].y();
    real_t left = -right;

    Transform3D M;

    M(0,0) = 2 * near / (right - left);
    M(1,0) = 0;
    M(2,0) = 0;
    M(3,0) = 0;

    M(0,1) = 0;
    M(1,1) = 2 * near / (top - bottom);
    M(2,1) = 0;
    M(3,1) = 0;

    M(0,2) = (right + left) / (right - left);
    M(1,2) = (top + bottom) / (top - bottom);
    M(2,2) = -(far + near) / (far - near);
    M(3,2) = -1;

    M(0,3) = 0;
    M(1,3) = 0;
    M(2,3) = -2 * far * near / (far - near);
    M(3,3) = 0;

    return M;
}

Scene::Scene(const pointBuffer_t& unitImage, bool useOpenGL, const real_t sceneRadius) : 
    unitImage_(unitImage),
    planeCount_(1), 
    sceneRadius_(sceneRadius),
    useOpenGLRendering_(useOpenGL),
    rcg_(HueSpread(10), SaturationSpread(64), ValueSpread(28))
{
    
    if (useOpenGLRendering_) {
        renderer_ = new OpenGLSceneRenderer(*this);
    } else {
        renderer_ = new RayTracingSceneRenderer(*this);
    }
}

Scene::Scene(Scene &&other) noexcept : 
    unitImage_(std::move(other.unitImage_)),
    primitives_(std::move(other.primitives_)), 
    cameraPose_(other.cameraPose_), 
    sceneRadius_(other.sceneRadius_),
    useOpenGLRendering_(other.useOpenGLRendering_),
    rcg_(std::move(other.rcg_))
{
    if (useOpenGLRendering_) {
        renderer_ = new OpenGLSceneRenderer(*this);
    } else {
        renderer_ = new RayTracingSceneRenderer(*this);
    }
}

void Scene::addPrimitive(rt::Primitive *primitive)
{
    // Plane id must be non-zero. Zero plane id indicates that a pixel does not belong to a plane.
    primitive->generateColorsAndPlaneIds(planeCount_, rcg_);
    primitives_.emplace_back(pPtr_t(primitive));
    const colormap_t& m = primitive->getColorPlaneMap();
    colorPlaneMap_.insert(m.begin(), m.end());
}

void Scene::draw() const
{
    for(const pPtr_t& p: primitives_){
        p->draw();
    }
}

void Scene::render(pointBuffer_t &pointBuffer, normalBuffer_t &normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t &depthBuffer, planeBuffer_t &planeBuffer)
{
    renderer_->render(pointBuffer, normalBuffer, colorBuffer, depthBuffer, planeBuffer);
}


void Scene::setCameraPose(const Transform3D &cameraPose)
{
    this->cameraPose_ = cameraPose;
    this->invCameraPose_ = cameraPose.inverse();
}

void Scene::setCameraPose(const real_t &x, const real_t &y, const real_t &z, const real_t &R, const real_t &P, const real_t &Y)
{
    populateTransform(x,y,z,R,P,Y,this->cameraPose_);
    this->invCameraPose_ = cameraPose_.inverse();
}

Transform3D Scene::getCameraPose() const
{
    return cameraPose_;
}

size_t Scene::getPlaneCount() const
{
    return planeCount_ - 1; // plane id starts from 1
}

const QColor &Scene::getPlaneColor(size_t planeId) const
{
    if (planeId == 0) {
        return noPlane;
    }
    return planeColors_[planeId - 1];
}


/*
* A scene with only one box, centered at the origin.
*/
scenePtr_t getSceneBox(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);
    rt::Primitive *aabb = new rt::Box(0.4, 2, 2, 2);
    Transform3D t_box;
    populateTransform(1,0,1,0,-0.8,0.5,t_box);
    aabb->applyTransform(t_box);
    s->addPrimitive(aabb);
    s->addPrimitive(getFloor());
    s->setCameraPose(x,y,z,R,P,Y);
    return s;
}

/*
* A scene with only one sphere, centered at the origin.
*/
scenePtr_t getSceneSphere(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);
    Transform3D t_sphere;
    populateTransform(1,0,1,0,0,0,t_sphere);
    rt::Primitive* sphere = new rt::Sphere(0.25, 1);
    sphere->applyTransform(t_sphere);
    s->addPrimitive(sphere);
    s->addPrimitive(getFloor());
    s->setCameraPose(x,y,z,R,P,Y);
    return s;
}

/*
* A scene with only one cone, centered at the origin.
*/
scenePtr_t getSceneCone(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);
    s->addPrimitive(new rt::Cone(3, 1, 3));
    return s;
}


/*
* A scene with only one cylinder, centered at the origing.
*/
scenePtr_t getSceneCylinder(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);
    s->addPrimitive(new rt::Cylinder(1, 3));
    return s;
}

/*
* A scene with only one circular disk, centered at the origin.
*/
scenePtr_t getSceneDisk(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_disk1;
    populateTransform(2,0.5,0,0,-0.8,0,t_disk1);
    rt::Primitive *disk1 = new rt::Disk(1);
    disk1->applyTransform(t_disk1);
    s->addPrimitive(disk1);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);
    return s;
}

/*
 * A scene with only one polygon.
*/
scenePtr_t getScenePolygon(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y)
{
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_polygon1;
    populateTransform(2,0,0.2,0,-0.8,-0.8,t_polygon1);

    rt::Polygon *p1 = new rt::Polygon(0.2);
    p1->addVertex(2,1).addVertex(2.5,2).addVertex(0.5,4).addVertex(-2.5,1.5).addVertex(-1.5,-4);
    p1->applyTransform(t_polygon1);
    s->addPrimitive(p1);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}


/*
 * A complex scene involving a cuboid and a sphere.
*/
scenePtr_t getSceneComplex1(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    rt::Primitive *aabb = new rt::Box(0.25, 2, 2, 2);
    Transform3D t_box;
    populateTransform(1,-0.25,1,0,-0.8,0.5,t_box);
    aabb->applyTransform(t_box);
    s->addPrimitive(aabb);

    Transform3D t_sphere;
    populateTransform(1,0.25,1,0,0,0,t_sphere);
    rt::Primitive* sphere = new rt::Sphere(0.25, 2);
    sphere->applyTransform(t_sphere);
    s->addPrimitive(sphere);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}

/* A scene with a box and a polygon. */
scenePtr_t getSceneComplex2(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    rt::Primitive *aabb = new rt::Box(0.4, 2, 2, 2);
    Transform3D t_box;
    populateTransform(1,0.5,1,0,-0.8,0.8,t_box);
    aabb->applyTransform(t_box);
    s->addPrimitive(aabb);

    Transform3D t_polygon;
    populateTransform(2,-0.5,0.2,0,-0.8,0,t_polygon);
    rt::Polygon *p = new rt::Polygon(0.2);
    p->addVertex(2,1).addVertex(2.5,2).addVertex(0.5,4).addVertex(-2.5,1.5).addVertex(-1.5,-4);
    p->applyTransform(t_polygon);
    s->addPrimitive(p);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}

/* A scene with three parallel polygons. */
scenePtr_t getSceneComplex3(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_polygon1;
    populateTransform(1,0,0.2,0,-0.8,-0.8,t_polygon1);
    rt::Polygon *p1 = new rt::Polygon(0.1);
    p1->addVertex(2,1).addVertex(2.5,2).addVertex(0.5,4).addVertex(-2.5,1.5).addVertex(-1.5,-4);
    p1->applyTransform(t_polygon1);
    s->addPrimitive(p1);

    Transform3D t_polygon2;
    populateTransform(1.5,0,0.2,0,-0.8,-0.8,t_polygon2);
    rt::Polygon *p2 = new rt::Polygon(0.2);
    p2->addVertex(2,1).addVertex(2.5,2).addVertex(0.5,4).addVertex(-2.5,1.5).addVertex(-1.5,-4);
    p2->applyTransform(t_polygon2);
    s->addPrimitive(p2);

    Transform3D t_polygon3;
    populateTransform(2.0,0,0.2,0,-0.8,-0.8,t_polygon3);
    rt::Polygon *p3 = new rt::Polygon(0.3);
    p3->addVertex(2,1).addVertex(2.5,2).addVertex(0.5,4).addVertex(-2.5,1.5).addVertex(-1.5,-4);
    p3->applyTransform(t_polygon3);
    s->addPrimitive(p3);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}

/* A scene with three spheres of different sizes. */
scenePtr_t getSceneComplex4(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_sphere1;
    populateTransform(1,-0.6,1,0,0,0,t_sphere1);
    rt::Primitive* sphere1 = new rt::Sphere(0.25, 1);
    sphere1->applyTransform(t_sphere1);
    s->addPrimitive(sphere1);

    Transform3D t_sphere2;
    populateTransform(1,0,1,0,0,0,t_sphere2);
    rt::Primitive* sphere2 = new rt::Sphere(0.2, 1);
    sphere2->applyTransform(t_sphere2);
    s->addPrimitive(sphere2);

    Transform3D t_sphere3;
    populateTransform(1,0.5,1,0,0,0,t_sphere3);
    rt::Primitive* sphere3 = new rt::Sphere(0.1, 1);
    sphere3->applyTransform(t_sphere3);
    s->addPrimitive(sphere3);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}

/* A scene with six differently oriented boxes. */
scenePtr_t getSceneComplex5(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_box1;
    populateTransform(1,-1,1,0.3,-0.8,0.5,t_box1);
    rt::Primitive *box1 = new rt::Box(0.2, 2, 2, 2);
    box1->applyTransform(t_box1);
    s->addPrimitive(box1);

    Transform3D t_box2;
    populateTransform(1,-0.5,1,-0.3,-0.8,-0.5,t_box2);
    rt::Primitive *box2 = new rt::Box(0.1, 2, 2, 2);
    box2->applyTransform(t_box2);
    s->addPrimitive(box2);

    Transform3D t_box3;
    populateTransform(1,0,1,0.9,0.2,-0.5,t_box3);
    rt::Primitive *box3 = new rt::Box(0.05, 2, 2, 2);
    box3->applyTransform(t_box3);
    s->addPrimitive(box3);

    Transform3D t_box4;
    populateTransform(1,0.5,1,-0.3,-0.8,-0.5,t_box4);
    rt::Primitive *box4 = new rt::Box(0.1, 2, 2, 2);
    box4->applyTransform(t_box4);
    s->addPrimitive(box4);

    Transform3D t_box5;
    populateTransform(1,1,1,0.3,0.8,0.5,t_box5);
    rt::Primitive *box5 = new rt::Box(0.2, 2, 2, 2);
    box5->applyTransform(t_box5);
    s->addPrimitive(box5);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}

/* A staircase scene. */
scenePtr_t getSceneComplex6(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    scenePtr_t s = std::make_shared<Scene>(unitImage, useOpenGL, sceneRadius);

    Transform3D t_box1;
    populateTransform(1,0,0,0,0,0,t_box1);
    rt::Primitive *box1 = new rt::Box(1, 0.2, 0.6, 0.2);
    box1->applyTransform(t_box1);
    s->addPrimitive(box1);

    Transform3D t_box2;
    populateTransform(1.2,0,0,0,0,0,t_box2);
    rt::Primitive *box2 = new rt::Box(1, 0.2, 0.6, 0.4);
    box2->applyTransform(t_box2);
    s->addPrimitive(box2);

    Transform3D t_box3;
    populateTransform(1.4,0,0,0,0,0,t_box3);
    rt::Primitive *box3 = new rt::Box(1, 0.2, 0.6, 0.6);
    box3->applyTransform(t_box3);
    s->addPrimitive(box3);

    Transform3D t_box4;
    populateTransform(1.6,0,0,0,0,0,t_box4);
    rt::Primitive *box4 = new rt::Box(1, 0.2, 0.6, 0.8);
    box4->applyTransform(t_box4);
    s->addPrimitive(box4);

    Transform3D t_box5;
    populateTransform(1.8,0,0,0,0,0,t_box5);
    rt::Primitive *box5 = new rt::Box(1, 0.2, 0.6, 1.0);
    box5->applyTransform(t_box5);
    s->addPrimitive(box5);

    s->addPrimitive(getFloor());

    s->setCameraPose(x,y,z,R,P,Y);

    return s;
}


scenePtr_t Scene::getScene(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const uchar &sceneId, const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y) {
    switch (sceneId) {
    case 1:
        return getSceneComplex1(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    case 2:
        return getSceneComplex2(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    case 3:
        return getSceneComplex3(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    case 4:
        return getSceneComplex4(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    case 5:
        return getSceneComplex5(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    case 6:
        return getSceneComplex6(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
    }

    return getSceneComplex1(unitImage, useOpenGL, sceneRadius, x, y, z, R, P, Y);
}

scenePtr_t Scene::getScene(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const uchar &sceneId, const Vec3& translation, const Vec3& rotation) {
    return Scene::getScene(unitImage, useOpenGL, sceneRadius, sceneId, translation.x(),translation.y(), translation.z(), rotation.x(), rotation.y(), rotation.z());
}


void Scene::RayTracingSceneRenderer::render ( pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer ) {
    // This is the camera point in world coordinates
    const Vec3 rayOrigin = outer_.cameraPose_ * origin_;

    // Find the inverse of the camera transformation
    const Transform3D& cameraPoseInv = outer_.cameraPose_.inverse();

    static Vec3 intersectionWorld, intersectionCamera, rayDirection, normalWorld, normalCamera;
    static Pixel color;
    static uint16_t depth;
    static size_t planeId;
    for (size_t i = 0; i < IMAGE_HEIGHT; i++) {
        for (size_t j = 0; j < IMAGE_WIDTH; j++) {

            const size_t idx = i * IMAGE_WIDTH + j;
            rayDirection = outer_.cameraPose_ * outer_.unitImage_[idx];
            rayDirection.normalize();

            for (const pPtr_t& primitive: outer_.primitives_) {

                intersectionWorld.fill(0); color.fill(0); normalWorld.fill(0); depth = 0;
                // Calculate the intersection point in the world frame.
                primitive->intersect(rayOrigin, rayDirection, intersectionWorld, normalWorld, color, depth, planeId);

                if (intersectionWorld.isZero() || intersectionWorld.array().isNaN().any()) {
                    continue;
                }

                Vec3& p = pointBuffer[idx];
                if (p.isZero() || p.array().isNaN().any() || depth < depthBuffer[idx]) {
                    // This is the first time this point is touched by a ray.
                    // Convert the intersection coordinates to the camera frame
                    intersectionCamera = cameraPoseInv * intersectionWorld;
                    normalCamera = cameraPoseInv.linear() * normalWorld;
                    p = intersectionCamera;
                    normalBuffer[idx] = normalCamera;
                    colorBuffer[idx] = color;
                    depthBuffer[idx] = depth;
                    planeBuffer[idx] = planeId;
                }
            }
        }
    }

    outer_.planeColors_.resize(outer_.planeCount_ - 1);
    for (size_t i = 0; i < outer_.planeCount_ - 1; i++) {
        outer_.planeColors_[i] = colorUtil.sampleUniformColor();
    }
}

pointBuffer_t initializeOpenGlUnitImage() {
    pointBuffer_t unitImage;

    double halfWidth = CAMERA_PLANE_DISTANCE*tan(0.5*CAMERA_OPENING_X*DEG_TO_RAD);
    double halfHeight = halfWidth / ASPECT_RATIO;
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            unitImage[i*IMAGE_WIDTH+j].x() = CAMERA_PLANE_DISTANCE;
            unitImage[i*IMAGE_WIDTH+j].y() = halfWidth - j * 2.0 * halfWidth / (IMAGE_WIDTH - 1.0);
            unitImage[i*IMAGE_WIDTH+j].z() = halfHeight - i * 2.0 * halfHeight / (IMAGE_HEIGHT - 1.0);
        }
    }

    return unitImage;
}

const pointBuffer_t Scene::OpenGLSceneRenderer::openGlUnitImage_ = initializeOpenGlUnitImage();

void Scene::OpenGLSceneRenderer::writeToPointBuffer(const float* zBuf, pointBuffer_t &pBuf)
{
    // Convert the depth values to distances.
    // To understand how this works, you need to understand how the OpenGL projection matrix
    // is made, what the z-divide is and what purpose it serves, and how to invert it to linear depth.
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/projection-matrix-introduction
    // https://learnopengl.com/Advanced-OpenGL/Depth-testing
    double near = CAMERA_PLANE_DISTANCE;
    double far = outer_.sceneRadius_;
    for (uint i = 0; i < NUMBER_OF_POINTS; i++)
    {
        real_t ndc = zBuf[i] * 2.0 - 1.0;
        real_t linearD = (2.0 * near * far) / (far + near - ndc * (far - near));
        pBuf[i] = (linearD/CAMERA_PLANE_DISTANCE) * openGlUnitImage_[i];
    }
    return;
}

void Scene::OpenGLSceneRenderer::render ( pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer ) {
    // Generate a framebuffer.
    static GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Prepare the color renderbuffer.
    static GLuint rboColor;
    glGenRenderbuffers(1, &rboColor);
    glBindRenderbuffer(GL_RENDERBUFFER, rboColor);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGB8, IMAGE_WIDTH, IMAGE_HEIGHT);
    glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rboColor);

    // Prepare the depth renderbuffer.
    static GLuint rboDepth;
    glGenRenderbuffers(1, &rboDepth);
    glBindRenderbuffer(GL_RENDERBUFFER, rboDepth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, IMAGE_WIDTH, IMAGE_HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, rboDepth);

    glPushAttrib(GL_VIEWPORT_BIT | GL_TRANSFORM_BIT);

    // Light setup
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Set up the projection and the model view matrices for the camera.
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glMultMatrixd(getFrustum(outer_.sceneRadius_, openGlUnitImage_).data());
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRotated(90, 0, 0, 1);
    glRotated(90, 0, 1, 0);
    glMultMatrixd(outer_.invCameraPose_.data());

    // Set up the view port.
    glViewport(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);

    // Draw the scene.
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    outer_.draw();
    glFlush();

    // Read the color buffer.
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, colorBuffer.data());

    // Flip it upside down.
    for (uint j = 0; j < IMAGE_HEIGHT/2; j++)
    {
        for (uint i = 0; i < IMAGE_WIDTH; i++)
        {
            uint a = i+j*IMAGE_WIDTH;
            uint b = i+(IMAGE_HEIGHT-1-j)*IMAGE_WIDTH;
            Pixel tmp = colorBuffer[a];
            colorBuffer[a] = colorBuffer[b];
            colorBuffer[b] = tmp;
        }
    }

    static float zBuffer[NUMBER_OF_POINTS];
    glReadBuffer(GL_DEPTH_ATTACHMENT);
    glReadPixels(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT, GL_DEPTH_COMPONENT, GL_FLOAT, zBuffer);

    // Flip it upside down.
    for (uint j = 0; j < IMAGE_HEIGHT/2; j++)
    {
        for (uint i = 0; i < IMAGE_WIDTH; i++)
        {
            uint a = i+j*IMAGE_WIDTH;
            uint b = i+(IMAGE_HEIGHT-1-j)*IMAGE_WIDTH;
            real_t tmp = zBuffer[a];
            zBuffer[a] = zBuffer[b];
            zBuffer[b] = tmp;
        }
    }

    // Convert the opengl z-buffer to a camera depth buffer
    writeToPointBuffer(zBuffer, pointBuffer);

    // Reset the buffers
    planeBuffer.fill(MAXVAL(size_t)); normalBuffer.fill(Vec3_0);

    const auto& normalTransform = outer_.cameraPose_.inverse().linear();

    // Now fill up the normal and plane buffers based on the color of the planes
    for (uint i = 0; i < NUMBER_OF_POINTS; i++) {
        const Pixel& color = colorBuffer[i];

        const size_t colorIndex = utils::rgbIndex(color);

        if (outer_.colorPlaneMap_.find(colorIndex) != outer_.colorPlaneMap_.end()) {
            const PlanePointNormal& plane = outer_.colorPlaneMap_[colorIndex];
            normalBuffer[i] = normalTransform * plane.n;
            planeBuffer[i] = plane.id;
        }
    }

    // Restore OpenGL state.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();

    // Clean up.
    glDeleteFramebuffers(1, &fbo);
    glDeleteRenderbuffers(1, &rboColor);
    glDeleteRenderbuffers(1, &rboDepth);    
}
