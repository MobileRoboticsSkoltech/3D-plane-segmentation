#ifndef SCENE_H
#define SCENE_H

#include <GL/glew.h>
#include <vector>
#include <memory>
#include <QColor>
#include <QGLViewer/qglviewer.h>
#include <unordered_map>

#include "primitive.h"
#include "utils/ColorUtil.h"
#include "globals/constants.h"

/*
 * An OpenGL scene. A scene can contain only a handful of primitives:
 * 1. Cuboid
 * 2. Sphere
 * 3. Cylider
 * 4. Cone
 * 5. Disk
 * 6. Polygon
*/

class Scene;

typedef std::unique_ptr<rt::Primitive> pPtr_t;
typedef std::shared_ptr<Scene> scenePtr_t; // redefinition

/*
 * This class encapsulates the scene rendering strategy.
 */
class SceneRenderer {

protected:

    Scene& outer_;

public:

    SceneRenderer(Scene& outer) : outer_(outer) {}

    virtual void render(pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer) = 0;

};

class Scene
{

private:
    
    /* Store a reference to the unit image. This is a constant for a given set of camera parameters. */
    const pointBuffer_t& unitImage_;

    /* The primitive objects in this scene. */
    std::vector<pPtr_t> primitives_;

    /* The location of the camera in the scene. */
    Transform3D cameraPose_, invCameraPose_;

    /* The world origin (0,0,0). */
    static const Vec3 origin_;

    /* Colors assigned to individual planes in the scene. */
    std::vector<QColor> planeColors_;

    /* Number of planes in the scene. */
    size_t planeCount_;
    
    /* The maximum distance the camera can see. */
    real_t sceneRadius_;
    
    /* A flag that determines whether to use OpenGL internal buffers or ray-tracing to render a scene. */
    bool useOpenGLRendering_;

    /* A random color generator for coloring the primitives. */
    RandomColorGenerator rcg_;

    /* This map is used to identify all planes in the scene by their colors. */
    colormap_t colorPlaneMap_;

    /* No plane marker color. */
    static const QColor noPlane;

    /* The rendering strategy in use. */
    SceneRenderer* renderer_;

    /* Use ray-tracing to render the scene. */
    class RayTracingSceneRenderer : public SceneRenderer {
    
    public:
        
        RayTracingSceneRenderer(Scene& outer) : SceneRenderer(outer) {} 
        
        void render(pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer) override;
        
    };
    
    /* Use OpenGL's internal buffers to render the scene. */
    class OpenGLSceneRenderer : public SceneRenderer, private QGLViewer {

    private:

        static const pointBuffer_t openGlUnitImage_;

        void writeToPointBuffer(const float *zBuf, pointBuffer_t& pBuf);
    
    public:
        
        OpenGLSceneRenderer(Scene& outer) : SceneRenderer(outer) {
            glewInit();
        }
        
        void render(pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer) override;
    
    };
    

public:

    Scene(const pointBuffer_t& unitImage, const bool useOpenGL = false, const real_t sceneRadius = 10);

    /*
     * Delete the copy constructors as the vector of
     * primitives is non-copyable.
    */
    Scene(const Scene& other) = delete;
    Scene& operator=(const Scene& other) = delete;

    /*
     * Instead incorporate move semantics.
    */
    Scene(Scene&& other) noexcept;
    Scene& operator=(Scene&& other) noexcept = delete;

    /* Add a primitive to the scene. */
    void addPrimitive(rt::Primitive* primitive);

    /* Draw the scene using OpenGL. */
    void draw() const;

    /*
     * Render this scene from the perspective of the camera defined by the transform.
     * Screen buffer is the target for the render. Each cell of the screen buffer
     * contains the nearest point (on an obstacle) along the ray cast from that
     * location. Call *setCameraPose* before calling this method to set the location
     * and orientation of the camera in the scene.
    */
    void render(pointBuffer_t& pointBuffer, normalBuffer_t& normalBuffer, colorBuffer_t& colorBuffer, depthBuffer_t& depthBuffer, planeBuffer_t& planeBuffer);

    /*
     * Set the pose of the camera in the scene.
    */
    void setCameraPose(const Transform3D& cameraPose);
    void setCameraPose(const real_t& x, const real_t& y, const real_t& z, const real_t& R, const real_t& P, const real_t& Y);

    /*
     * Get the pose of the camera in the scene.
    */
    Transform3D getCameraPose() const;

    /* Get the total number of planes in the scene. */
    size_t getPlaneCount() const;

    /* Get the color assigned to this planeId. */
    const QColor& getPlaneColor(size_t planeId) const;

    /* Get some pre-constructed scenes. */
    static scenePtr_t getScene(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const uchar &sceneId, const real_t &x, const real_t &y, const real_t &z, const real_t &R, const real_t &P, const real_t &Y);
    static scenePtr_t getScene(const pointBuffer_t& unitImage, const bool useOpenGL, const real_t sceneRadius, const uchar &sceneId, const Vec3& translation, const Vec3& rotation);

};

#endif // SCENE_H
