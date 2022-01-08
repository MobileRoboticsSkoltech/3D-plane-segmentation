#ifndef GLLIB
#define GLLIB

#include "globals/constants.h"

#include <QColor>
#include <GL/glut.h>

#ifndef CALLBACK
#define CALLBACK
#endif

#ifndef _WIN32
#define __stdcall
#endif

namespace GLlib
{
	// 1: top left front
    // 2: bottom left front
	// 3: top right front
    // 4: bottom right front
	// 5: top right back
    // 6: bottom right back
	// 7: top left back
    // 8: bottom left back
	void drawBox(
		double x1, double y1, double z1,
		double x2, double y2, double z2,
		double x3, double y3, double z3,
		double x4, double y4, double z4,
		double x5, double y5, double z5,
		double x6, double y6, double z6,
		double x7, double y7, double z7,
		double x8, double y8, double z8
		);

    void drawBox(
            const Vec3& tlf, const Vec3& dlf,
            const Vec3& trf, const Vec3& drf,
            const Vec3& trb, const Vec3& drb,
            const Vec3& tlb, const Vec3& dlb,
            const QColor& top, const QColor& down,
            const QColor& left, const QColor& right,
            const QColor& front, const QColor& back);

    // Draws a cuboid with half extents x, y, and z.
    void drawBox(float x=0.5, float y=0.5, float z=0.5);

    // Draws a cone.
    void drawCone(float bottomRadius, float topRadius, float height);

    // Draws a cylinder.
    void drawCylinder(float radius, float height);

    // Draws a sphere.
    void drawSphere(float radius = 1.0);

    // Draws a circle in the xy plane.
    void drawCircle(float radius = 0.1);
    void drawFilledCircle(float radius = 0.1);

    /************Functions for drawing non-convex polygons.************/

    // From http://www.songho.ca/opengl/gl_tessellation.html

    // CALLBACK functions for GLU_TESS ////////////////////////////////////////////
    // NOTE: must be declared with CALLBACK directive
    void CALLBACK tessBeginCB(GLenum which);
    void CALLBACK tessEndCB();
    void CALLBACK tessErrorCB(GLenum errorCode);
    void CALLBACK tessVertexCB(const GLvoid *data);

    void drawPolygon(const std::vector<Vec3>& points);

}

#endif //GLLIB
