#include "utils/GLlib.h"
#include <math.h>
#include <GL/glu.h>

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
                    )
    {
        glBegin(GL_QUAD_STRIP);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glVertex3f(x3, y3, z3);
        glVertex3f(x4, y4, z4);
        glVertex3f(x5, y5, z5);
        glVertex3f(x6, y6, z6);
        glVertex3f(x7, y7, z7);
        glVertex3f(x8, y8, z8);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glEnd();

        glBegin(GL_QUAD_STRIP);
        glVertex3f(x1, y1, z1);
        glVertex3f(x3, y3, z3);
        glVertex3f(x5, y5, z5);
        glVertex3f(x7, y7, z7);
        glVertex3f(x2, y2, z2);
        glVertex3f(x4, y4, z4);
        glVertex3f(x6, y6, z6);
        glVertex3f(x8, y8, z8);
        glEnd();

        glColor3f(0.3, 0.3, 0.3);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
        glVertex3f(x4, y4, z4);
        glVertex3f(x3, y3, z3);
        glVertex3f(x1, y1, z1);
        glVertex3f(x7, y7, z7);
        glVertex3f(x8, y8, z8);
        glVertex3f(x6, y6, z6);
        glVertex3f(x5, y5, z5);
        glVertex3f(x7, y7, z7);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(x3, y3, z3);
        glVertex3f(x5, y5, z5);
        glVertex3f(x4, y4, z4);
        glVertex3f(x6, y6, z6);
        glVertex3f(x2, y2, z2);
        glVertex3f(x8, y8, z8);
        glEnd();
    }

    // Draws a cuboid with half extents x, y and z.
    void drawBox(float x, float y, float z)
    {
        glBegin( GL_QUAD_STRIP );
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, y, -z);
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glEnd();

        glBegin( GL_QUADS );
        glVertex3f(x, y, z);
        glVertex3f(x, -y, z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, y, -z);
        glEnd();

        glColor3f(0, 0, 0);
        glLineWidth(2);
        glBegin( GL_LINE_STRIP );
        glVertex3f(x, y, z);
        glVertex3f(x, y, -z);
        glVertex3f(x, -y, -z);
        glVertex3f(x, -y, z);
        glVertex3f(x, y, z);
        glVertex3f(-x, y, z);
        glVertex3f(-x, y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(-x, -y, z);
        glVertex3f(-x, y, z);
        glEnd();

        glBegin( GL_LINES );
        glVertex3f(x, -y, z);
        glVertex3f(-x, -y, z);
        glVertex3f(x, -y, -z);
        glVertex3f(-x, -y, -z);
        glVertex3f(x, y, -z);
        glVertex3f(-x, y, -z);
        glEnd();
    }

    // Draws a sphere.
    void drawSphere(float radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a sphere with a camera plane aligned border around it (you wish).
    void drawBorderedSphere(float radius)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluSphere(quadric, radius, 32, 32);
    }

    // Draws a circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawCircle(float radius)
    {
        int slices = 32;

        float theta = 2 * 3.1415926 / float(slices);
        float c = cos(theta);
        float s = sin(theta);
        float t;

        float x = radius; //we start at angle = 0
        float y = 0;

        glLineWidth(2);
        glColor3f(0, 0 , 0);
        glBegin(GL_LINE_LOOP);
        for(int ii = 0; ii < slices; ii++)
        {
            glVertex3f(x, y, 0);

            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
        glEnd();
    }

    // Draws a filled circle in the xy plane.
    // http://slabode.exofire.net/circle_draw.shtml
    void drawFilledCircle(float radius)
    {
        int slices = 32;

        float theta = 2 * 3.1415926 / float(slices);
        float c = cos(theta);
        float s = sin(theta);
        float t;

        float x = radius; // start at angle = 0
        float y = 0;

        glBegin(GL_TRIANGLE_FAN);
        glVertex3f(0, 0, 0);
        for(int ii = 0; ii <= slices; ii++)
        {
            glVertex3f(x, y, 0);
            t = x;
            x = c*x-s*y; // rotation matrix
            y = s*t+c*y; // rotation matrix
        }
        glEnd();
    }

    void drawCone(float bottomRadius, float topRadius, float height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, bottomRadius, topRadius, height, 32, 32);
        drawFilledCircle(bottomRadius);
        glTranslated(0, 0, height);
        drawFilledCircle(topRadius);
        drawCircle(topRadius);
        glTranslated(0, 0, -height);
        drawCircle(bottomRadius);
    }

    void drawCylinder(float radius, float height)
    {
        static GLUquadric* quadric = gluNewQuadric();
        gluCylinder(quadric, radius, radius, height, 32, 32);
        drawFilledCircle(radius);
        glTranslated(0, 0, height);
        drawFilledCircle(radius);
        drawCircle(radius);
        glTranslated(0, 0, -height);
        drawCircle(radius);
    }

    void drawQuad(const Vec3 &v1, const Vec3 &v2, const Vec3 &v3, const Vec3 &v4, const QColor &color)
    {
        glColor3ub(color.red(), color.green(), color.blue());

        glBegin(GL_QUADS);
        glVertex3f(v1.x(), v1.y(), v1.z());
        glVertex3f(v2.x(), v2.y(), v2.z());
        glVertex3f(v3.x(), v3.y(), v3.z());
        glVertex3f(v4.x(), v4.y(), v4.z());
        glEnd();
    }

    void drawBox(
            const Vec3 &tlf, const Vec3 &dlf,
            const Vec3 &trf, const Vec3 &drf,
            const Vec3 &trb, const Vec3 &drb,
            const Vec3 &tlb, const Vec3 &dlb,
            const QColor &top, const QColor &down,
            const QColor &left, const QColor &right,
            const QColor &front, const QColor &back)
    {
        // Draw front face
        drawQuad(tlf, trf, drf, dlf, front);

        // Draw back face
        drawQuad(tlb, trb, drb, dlb, back);

        // Draw top face
        drawQuad(tlf, tlb, trb, trf, top);

        // Draw down face
        drawQuad(dlf, dlb, drb, drf, down);

        // Draw left face
        drawQuad(tlf, tlb, dlb, dlf, left);

        // Draw right face
        drawQuad(trf, trb, drb, drf, right);
    }

    void drawPolygon(const std::vector<Vec3> &points)
    {
        GLUtesselator *tess = gluNewTess(); // create a tessellator

        if(!tess){
            return;  // failed to create tessellation object
        }

        // register callback functions
        gluTessCallback(tess, GLU_TESS_BEGIN, (void (CALLBACK *)())tessBeginCB);
        gluTessCallback(tess, GLU_TESS_END, (void (CALLBACK *)())tessEndCB);
        gluTessCallback(tess, GLU_TESS_ERROR, (void (CALLBACK *)())tessErrorCB);
        gluTessCallback(tess, GLU_TESS_VERTEX, (void (CALLBACK *)())tessVertexCB);

        gluTessBeginPolygon(tess, 0);                   // with NULL data
            gluTessBeginContour(tess);
                for (auto &p: points) {
                    gluTessVertex(tess, (GLdouble*)p.data(), (GLvoid*) p.data());
                }
            gluTessEndContour(tess);
        gluTessEndPolygon(tess);

        gluDeleteTess(tess);        // safe to delete after tessellation
    }

    ///////////////////////////////////////////////////////////////////////////////
    // GLU_TESS CALLBACKS
    ///////////////////////////////////////////////////////////////////////////////
    void CALLBACK tessBeginCB(GLenum which)
    {
        glBegin(which);
    }

    void CALLBACK tessEndCB()
    {
        glEnd();
    }

    void CALLBACK tessVertexCB(const GLvoid *data)
    {
        // cast back to double type
        const GLdouble *ptr = (const GLdouble*)data;

        glVertex3dv(ptr);
    }

    void CALLBACK tessErrorCB(GLenum errorCode)
    {
        const GLubyte *errorStr;

        errorStr = gluErrorString(errorCode);
    }

}
