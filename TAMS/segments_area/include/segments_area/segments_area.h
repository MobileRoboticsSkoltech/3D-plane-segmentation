/*
 * Software License Agreement (BSD License)
 *
 *  Technical Aspects of Multimodal Systems (TAMS) - http://tams-www.informatik.uni-hamburg.de/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TAMS, nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Junhao Xiao
 * Email  : junhao.xiao@ieee.org, xiao@informatik.uni-hamburg.de
 *
 */

#ifndef SEGMENTS_AREA_H_
#define SEGMENTS_AREA_H_

//STL
#include <vector>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//CGAL
// CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
//boost
#include <boost/shared_ptr.hpp>
//tams
#include "segments_area/segments_area.h"
#include "common/planar_patch.h"

namespace tams
{
  /** \brief This class is used to calculate the area of each planar segment resulted from plane segmentation of a 3D point cloud.
    * Four methods are provides, namely, surface integrals inspired (proposed by us), Delaunay triangulation based, Alpha-shapes based,
    * and the number of square units (see )*/
  /** \todo add references. */
  class SegmentsArea
  {
      typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
      typedef K::Point_2 CGALPoint2;
      typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;
      typedef CGAL::Alpha_shape_face_base_2<K>  Fb;
      typedef CGAL::Triangulation_data_structure_2<Vb,Fb> Tds;
      typedef CGAL::Delaunay_triangulation_2<K, Tds> Delaunay;
      typedef CGAL::Alpha_shape_2<Delaunay> Alpha_shape_2;

    public:
      enum AreaCalculationMethod {SumOfSmallFaces, DelaunayTriangulation, AlphaShape, NumberOfSquareUnits};

      /** \brief Empty constructor. */
      SegmentsArea();
      /** \brief Constructor with point cloud, segments, method and sensor resolution.
        * \param[in] cloud boost shared pointer to the corresponding point cloud
        * \param[in] planar segments from a plane segmentation of cloud
        * \param[in] the method to use, four methods have been provided, namely
        *            SumOfSmallFaces, DelaunayTriangulation, AlphaShape, and NumberOfSquareUnits.
        * \param[in] vertical_resolution vertical resolution of the scanner wich was used to scan the given cloud
        * \param[in] horizontal_resolution horizontal resolution of the scanner wich was used to scan the given cloud
        */
      SegmentsArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                   PlanarSegment::StdVectorPtr segments,
                   AreaCalculationMethod method = SumOfSmallFaces,
                   double vertical_resolution = 0.0,
                   double horizontal_resolution = 0.0);

      /** \brief Calculate the surface area by summing the area of small surfaces.
        * \param[in] cloud boost shared pointer to the corresponding point cloud.
        * \param[in] segments planar segments from a plane segmentation result of cloud
        */
      void
      areaBySumOfSmallFaces(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments);

      /** \brief Calculate the surface area by summing the area of each triangle results from Delaunay triangulation.
        * \param[in] cloud boost shared pointer to the corresponding point cloud.
        * \param[in] segments planar segments from a plane segmentation result of cloud
        */
      void
      areaByDelaunayTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments);

      /** \brief Calculate the surface area by summing the area of each triangle results from Alpha shape.
        * \param[in] cloud boost shared pointer to the corresponding point cloud.
        * \param[in] segments planar segments from a plane segmentation result of cloud
        */
      void
      areaByAlphaShape(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments);

      /** \brief Calculate the surface area by counting the number of square units in the segment.
        * \param[in] cloud boost shared pointer to the corresponding point cloud.
        * \param[in] segments planar segments from a plane segmentation result of cloud
        */
      void
      areaByNumberOfSquareUnits(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments);

    private:
      /** \brief Transfer pcl::PointXYZ to Eigen::vector3d which is easy for computation.
        * \param[in] cloud boost shared pointer to the corresponding point cloud.
        */
      void
      toEigenTypes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

      /** \brief Project the points from spatial coordinates (3D) to a planar coordinates (2D).
        * The segment is rotated after what its surface normal is aligned to z-axis.
        * The translation is not considered here, since the z-coordinate will be omitted.
        * \param[in] segment a planar segment which will be projected to a planar coordinate.
        * \param[out] cgal_points the resulted 2D points in the CGAL data format
        */
      void
      projectSegmentTo2D(PlanarSegment segment, std::vector<CGALPoint2> &cgal_points);

    private:
      std::vector <Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points_;
      double vertical_resolution_;
      double horizontal_resolution_;
      bool verbose_;
  };
}

#endif
