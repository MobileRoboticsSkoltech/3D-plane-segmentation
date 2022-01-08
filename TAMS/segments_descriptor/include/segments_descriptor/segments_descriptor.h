#ifndef SEGMENTS_DESCRIPTOR_H_
#define SEGMENTS_DESCRIPTOR_H_
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "common/planar_patch.h"
#include <algorithm>
namespace tams
{
  struct FeatureBasedOnSegment
  {
    double volume;
    double max_volume;
    double normalized_volume;
    std::vector<double> orientation_hist;
    double
    distance(FeatureBasedOnSegment &rhs)
    {
      double sum = 0.0;
      for (size_t i = 0; i < orientation_hist.size (); i++)
      {
        sum += orientation_hist[i];
      }
      double rhs_sum = 0.0;
      for (size_t i = 0; i < orientation_hist.size (); i++)
      {
        rhs_sum += rhs.orientation_hist[i];
      }
      double dis = 0.0;
      size_t count = 0;
      for (size_t i = 0; i < orientation_hist.size (); i++)
      {
        if ((orientation_hist[i] > 0.0) || (rhs.orientation_hist[i] > 0.0))
        {
          dis += (orientation_hist[i]/sum - rhs.orientation_hist[i]/rhs_sum) *
              (orientation_hist[i]/sum - rhs.orientation_hist[i]/rhs_sum);
        }
      }
      dis = sqrt(dis) *
          (std::max(sum, rhs_sum) / std::min(sum, rhs_sum)) *
          (std::max(sum, rhs_sum) / std::min(sum, rhs_sum));
      //dis = sqrt(dis) * (std::max(volume, rhs.volume) / std::min(volume, rhs.volume))
      //    * (std::max(sum, rhs_sum) / std::min(sum, rhs_sum));
      //std::cout << std::max(sum, rhs_sum) / std::min(sum, rhs_sum) << "\t";
      return dis;
/*      double sum = 0.0;
      double rhs_sum = 0.0;
      double dot_product = 0.0;
      double self_dot_product = 0.0;
      double rhs_dot_product = 0.0;
      for (size_t i = 0; i < orientation_hist.size(); i++)
      {
        sum += orientation_hist[i];
        rhs_sum += rhs.orientation_hist[i];
        dot_product += orientation_hist[i]*rhs.orientation_hist[i];
        self_dot_product += orientation_hist[i]*orientation_hist[i];
        rhs_dot_product += rhs.orientation_hist[i]*rhs.orientation_hist[i];
      }
      return (1 - (dot_product * dot_product) / (self_dot_product * rhs_dot_product)) *
          std::max(orientation_hist[0],rhs.orientation_hist[0]) / std::min(orientation_hist[0],rhs.orientation_hist[0]) *
          std::max(orientation_hist[0],rhs.orientation_hist[0]) / std::min(orientation_hist[0],rhs.orientation_hist[0]) *
          std::max(sum, rhs_sum) * std::max(sum, rhs_sum) / std::min(sum, rhs_sum) / std::min(sum, rhs_sum) ;*/
    }
  };
  using namespace Eigen;
  template<typename PointT>
  class SegmentsDescriptor : public pcl::PCLBase<PointT>
  {
    using pcl::PCLBase<PointT>::initCompute;
    using pcl::PCLBase<PointT>::deinitCompute;
    using pcl::PCLBase<PointT>::input_;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

  public:
    double volume_;
    double normalized_volume_;
    SegmentsDescriptor () :
      alpha_(0.7), min_planar_patch_area_ (0.25), area_sum_ (0.0)
    {
      //cubeFaces ();
      //icosahedronVertices ();
      novelDirections ();
    }
    void
    setSegments(std::vector<PlanarSegment, aligned_allocator<PlanarSegment> > &segments)
    {
      segments_.clear ();
      segments_ = segments;
    }
    void
    computeAppearances(std::vector<FeatureBasedOnSegment> &appearances);

    void
    appearancesVisualization(pcl::visualization::PCLVisualizer *pViewer,
                             std::vector<FeatureBasedOnSegment> &_appearances);
  private:
    void
    clustering ();
    void
    orientation_invariant();
    void
    cubeFaces ()
    {
      orientations_.clear();
      orientations_.push_back(Vector3d(1, 0, 0).normalized());
      orientations_.push_back(Vector3d(0, 1, 0).normalized());
      orientations_.push_back(Vector3d(0, 0, 1).normalized());
      orientations_.push_back(Vector3d(-1, 0, 0).normalized());
      orientations_.push_back(Vector3d(0, -1, 0).normalized());
      orientations_.push_back(Vector3d(0, 0, -1).normalized());
    }
    void
    cubeVerticesAndFaces()
    {
      std::vector<Vector3d, aligned_allocator<Vector3d> > cubeVertices;
      cubeVertices.push_back(Vector3d(-1, -1, -1).normalized());
      cubeVertices.push_back(Vector3d(-1, -1,  1).normalized());
      cubeVertices.push_back(Vector3d(-1,  1, -1).normalized());
      cubeVertices.push_back(Vector3d(-1,  1,  1).normalized());
      cubeVertices.push_back(Vector3d( 1, -1, -1).normalized());
      cubeVertices.push_back(Vector3d( 1, -1,  1).normalized());
      cubeVertices.push_back(Vector3d( 1,  1, -1).normalized());
      cubeVertices.push_back(Vector3d( 1,  1,  1).normalized());
      std::vector<Vector3d, aligned_allocator<Vector3d> > cubeFaces;
      cubeFaces.push_back(Vector3d(0, 0, 1));
      cubeFaces.push_back(Vector3d(0, 1, 0));
      cubeFaces.push_back(Vector3d(1, 0, 0));
      cubeFaces.push_back(Vector3d(0, 0, -1));
      cubeFaces.push_back(Vector3d(0, -1, 0));
      cubeFaces.push_back(Vector3d(-1, 0, 0));

      orientations_.clear();
      orientations_.insert(orientations_.begin(), cubeVertices.begin(), cubeVertices.end());
      orientations_.insert(orientations_.begin(), cubeFaces.begin(), cubeFaces.end());
    }
    void
    novelDirections ()
    {
      std::vector<Vector3d, aligned_allocator<Vector3d> > directions;
      directions.clear ();
      directions.push_back(Vector3d( 0.000000,  0.000000,  1.000000));
      double height = cos(63.4350 * M_PI / 180);
      for (int i = 0; i < 36; i++)
      {
        Vector2d n(cos(i * 10 * M_PI / 180), sin(i * 10 * M_PI / 180));
        n = n.normalized() * sqrt(1 - height * height);
        directions.push_back(Vector3d(n(0), n(1), height));
      }
      for (int i = 0; i < 36; i++)
      {
        Vector2d n(cos(i * 10 * M_PI / 180), sin(i * 10 * M_PI / 180));
        directions.push_back(Vector3d(n(0), n(1), 0.0));
      }

//      directions.push_back(Vector3d( 0.894427,  0.000000,  0.447214));
//      directions.push_back(Vector3d( 0.632455,  0.632455,  0.447214));
//      directions.push_back(Vector3d( 0.000000,  0.894427,  0.447214));
//      directions.push_back(Vector3d(-0.632455,  0.632455,  0.447214));
//      directions.push_back(Vector3d(-0.894427,  0.000000,  0.447214));
//      directions.push_back(Vector3d(-0.632455, -0.632455,  0.447214));
//      directions.push_back(Vector3d( 0.000000, -0.894427,  0.447214));
//      directions.push_back(Vector3d( 0.632455, -0.632455,  0.447214));
//
//      directions.push_back(Vector3d( 1.000000,  0.000000,  0.000000));
//      directions.push_back(Vector3d( 1.000000,  0.000000,  0.000000));
//      directions.push_back(Vector3d( 0.707107,  0.707107,  0.000000));
//      directions.push_back(Vector3d( 0.000000,  1.000000,  0.000000));
//      directions.push_back(Vector3d(-0.707107,  0.707107,  0.000000));
//
//      directions.push_back(Vector3d(-1.000000,  0.000000,  0.000000));
//      directions.push_back(Vector3d(-0.707107, -0.707107,  0.000000));
//      directions.push_back(Vector3d( 0.000000, -1.000000,  0.000000));
//      directions.push_back(Vector3d( 0.707107, -0.707107,  0.000000));

      orientations_.clear ();
      orientations_ = directions;
    }
    void
    icosahedronVertices ()
    {
      /** \brief Get the 12 directions which will be used as cluster center.
       * The directions are normalized vertices of a icosahedron, so they are
       * even distributed on a unit sphere. The following Cartesian coordinates
       * define the vertices of an icosahedron with edge-length 2, centered at the origin:
       *                        (0, ±1, ±φ)
       *                        (±1, ±φ, 0)
       *                        (±φ, 0, ±1).
       * For more detail, please see http://en.wikipedia.org/wiki/Icosahedron.
       */
      std::vector<Vector3d, aligned_allocator<Vector3d> > icosahedron_vertices;

      icosahedron_vertices.clear ();
      icosahedron_vertices.push_back (Vector3d( 0.000000,  0.000000,  1.000000));
      icosahedron_vertices.push_back (Vector3d( 0.000000, -0.894427,  0.447214));
      icosahedron_vertices.push_back (Vector3d( 0.525731,  0.723607,  0.447214));
      icosahedron_vertices.push_back (Vector3d(-0.525731,  0.723607,  0.447214));
      icosahedron_vertices.push_back (Vector3d( 0.850651, -0.276393,  0.447214));
      icosahedron_vertices.push_back (Vector3d(-0.850651, -0.276393,  0.447214));

      icosahedron_vertices.push_back (Vector3d(0.0000000,  0.000000, -1.000000));
      icosahedron_vertices.push_back (Vector3d(0.0000000,  0.894427, -0.447214));
      icosahedron_vertices.push_back (Vector3d(-0.525731, -0.723607, -0.447214));
      icosahedron_vertices.push_back (Vector3d( 0.525731, -0.723607, -0.447214));
      icosahedron_vertices.push_back (Vector3d(-0.850651,  0.276393, -0.447214));
      icosahedron_vertices.push_back (Vector3d( 0.850651,  0.276393, -0.447214));

      orientations_.clear ();
      orientations_ = icosahedron_vertices;
/*
 *    double phi = (1 + sqrt (5)) / 2;
      icosahedron_vertices.push_back (Vector3d(0.0, 1.0, phi).normalized ());
      icosahedron_vertices.push_back (Vector3d(0.0, 1.0, -phi).normalized ());
      icosahedron_vertices.push_back (Vector3d(0.0, -1.0, phi).normalized ());
      icosahedron_vertices.push_back (Vector3d(0.0, -1.0, -phi).normalized ());
      icosahedron_vertices.push_back (Vector3d(1.0, phi, 0.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(1.0, -phi, 0.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(-1.0, phi, 0.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(-1.0, -phi, 0.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(phi, 0.0, 1.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(-phi, 0.0, 1.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(phi, 0.0, -1.0).normalized ());
      icosahedron_vertices.push_back (Vector3d(-phi, 0.0, -1.0).normalized ());

      Vector3d axis_1 = icosahedron_vertices[0].cross (Vector3d::UnitZ ());
      axis_1 = axis_1.normalized ();
      double angle_1 = acos (icosahedron_vertices[0].dot (Vector3d::UnitZ ()));
      AngleAxis<double> rotation_1 (angle_1, axis_1);
      Matrix3d rotation_matrix_1 = rotation_1.toRotationMatrix ();
      Vector3d axis_2 = Vector3d::UnitZ ();
      Vector3d tmp_direction = rotation_matrix_1 * icosahedron_vertices[1];
      tmp_direction(2) = 0;
      tmp_direction = tmp_direction.normalized ();
      double angle_2 = acos (tmp_direction.dot(Vector3d::UnitY ()));
      AngleAxis<double> rotation_2 (angle_2, axis_2);
      Matrix3d rotation_matrix_2 = rotation_2.toRotationMatrix ();
      Matrix3d rotation_matrix = rotation_matrix_2 * rotation_matrix_1;
      for (size_t i = 0; i < icosahedron_vertices.size (); i++)
      {
        icosahedron_vertices[i] = rotation_matrix * icosahedron_vertices[i];
        std::cout << icosahedron_vertices[i](0) << "\t" <<
                     icosahedron_vertices[i](1) << "\t" <<
                     icosahedron_vertices[i](2) << std::endl;
       }
      orientations_.clear ();
      orientations_ = icosahedron_vertices;
*/
    }
  private:
    double alpha_;
    /** Planar patches have smaller area than min_planar_patch_area_ will be considered
     * as noisy ones.
     */
    double min_planar_patch_area_;
    double area_sum_;

    std::vector<Vector3d, aligned_allocator<Vector3d> > orientations_;
    std::vector<PlanarSegment, aligned_allocator<PlanarSegment> > segments_;
    std::vector<PlanarSegment> noisy_segments_;
    std::vector<PlanarSegment> saliency_segments_;
    std::vector<std::pair<double, Vector3d>, aligned_allocator<std::pair<double, Vector3d> > > area_at_directions_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
#endif
