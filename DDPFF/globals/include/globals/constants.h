#ifndef GLOBALS_H
#define GLOBALS_H

#include <math.h>
#include <cstdint>
#include <limits>
#include <array>
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#define STATS 1
#define UNUSED(expr) (void)(expr) // used to suppress unused warnings
#define MAXVAL(U) std::numeric_limits<U>::max()

using real_t = double;
using integer_t = int;
using unsigned_t = size_t;
using depth_t = uint16_t;

inline const real_t PI = 3.1415926535897932384626433832795;
inline const real_t RAD_TO_DEG =  180.0 / PI;
inline const real_t DEG_TO_RAD =  PI / 180.0;
inline const real_t EPSILON = 1.0E-6;
inline const real_t MAXREAL = std::numeric_limits<real_t>::max();
inline const int IMAGE_WIDTH = 640;
inline const int IMAGE_HEIGHT = 480;
inline const int CAMERA_SAMPLE_READ_WAIT_TIMEOUT = 2000; // in ms
inline const int CAMERA_FPS = 30;
inline const real_t CAMERA_OPENING_X = 60;
inline const real_t CAMERA_OPENING_Y = 46;
inline const real_t CAMERA_OPENING_TAN_X(2 * tan(0.5 * CAMERA_OPENING_X * DEG_TO_RAD));
inline const real_t CAMERA_OPENING_TAN_Y(2 * tan(0.5 * CAMERA_OPENING_Y * DEG_TO_RAD));
inline const real_t CAMERA_PLANE_DISTANCE = 0.1;
inline const real_t ASPECT_RATIO = IMAGE_WIDTH / IMAGE_HEIGHT;

inline const int SAMPLE_FACTOR = 1;
inline const int NUMBER_OF_POINTS = IMAGE_HEIGHT*IMAGE_WIDTH;

using Vec3 = Eigen::Matrix<real_t,3,1>;
using Vec2 = Eigen::Matrix<real_t,2,1>;
using Vec2u = Eigen::Matrix<unsigned_t,2,1>;
using Vec2i = Eigen::Matrix<integer_t,2,1>;
using Rot = Eigen::Quaternion<real_t>;
using Trans = Eigen::Translation<real_t, 3>;
using Transform3D = Eigen::Transform<real_t, 3, Eigen::Affine>;
using Pixel = Eigen::Matrix<unsigned char,3,1>;
typedef std::array<Vec3, NUMBER_OF_POINTS> pointBuffer_t;
typedef std::array<Vec3, NUMBER_OF_POINTS> normalBuffer_t;
typedef std::array<Pixel, NUMBER_OF_POINTS> colorBuffer_t;
typedef std::array<depth_t, NUMBER_OF_POINTS> depthBuffer_t;
typedef std::array<unsigned_t, NUMBER_OF_POINTS> planeBuffer_t;
inline const Vec3 Vec3_0 = Vec3::Zero();
inline const Vec2 Vec2_0 = Vec2::Zero();
inline const Vec2u Vec2u_0 = Vec2u::Zero();
inline const Vec2i Vec2i_0 = Vec2i::Zero();
inline const Pixel White = Pixel(255,255,255);
inline const Pixel Black = Pixel(0,0,0);

#endif // GLOBALS_H
