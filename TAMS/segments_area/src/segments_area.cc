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

//Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
//STL
#include <iostream>
#include <fstream>
#include <sys/time.h>
//tams
#include "segments_area/segments_area.h"

namespace tams
{
  SegmentsArea::SegmentsArea()
  {

  }


  SegmentsArea::SegmentsArea(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             PlanarSegment::StdVectorPtr segments,
                             AreaCalculationMethod method,
                             double vertical_resolution,
                             double horizontal_resolution)
  {
    verbose_ = true;
    vertical_resolution_ = vertical_resolution;
    horizontal_resolution_ = horizontal_resolution;
    toEigenTypes (cloud);
    switch (method)
    {
      case SumOfSmallFaces:
        areaBySumOfSmallFaces (cloud, segments);
        break;
      case DelaunayTriangulation:
        areaByDelaunayTriangulation (cloud, segments);
        break;
      case AlphaShape:
        areaByAlphaShape (cloud, segments);
        break;
      case NumberOfSquareUnits:
        if (vertical_resolution_ == 0.0 || horizontal_resolution_ == 0.0)
        {
          std::cerr << "Please give the sensor resolution as argument in order to use the method NumberOfSquareUnits.\n";
          return;
        }
        areaByNumberOfSquareUnits (cloud, segments);
        break;
      default:
        std::cerr << "please choose a method from SumOfSmallFaces, DelaunayTriangulation, AlphaShape, NumberOfSquareUnits. \n";
        break;
    }
  }

  void
  SegmentsArea::areaBySumOfSmallFaces (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments)
  {
    ///pos_index - width -1     pos_index - 1  pos_index -1 + width
    ///pos_index - width        pos_index      pos_index + width
    ///pos_index + 1 - width    pos_index + 1  pos_index + width + 1
    if (cloud->height == 1)
      std::cerr << "Sorry, this method can only be employed for organized point clouds.\n";

    struct timeval tpstart,tpend;
    double timeuse;
    //start the timer
    std::ofstream area_calculation_time;
    if (verbose_)
    {
      area_calculation_time.open("area_calculation_time.txt", std::fstream::app);
      gettimeofday(&tpstart,NULL);
    }

    int height = cloud->height;
    int width = cloud->width;
    std::vector<Eigen::Vector3d, aligned_allocator<Eigen::Vector3d> > cross_products;
    cross_products.resize(segments->size(), Eigen::Vector3d::Zero());
    int *flags = new int [width * height];
    memset(flags, 9999, width * height * sizeof(int));
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      for (std::vector<int>::iterator sub_it = it->points.begin(); sub_it != it->points.end (); sub_it++)
      {
        flags[*sub_it] = it - segments->begin();
      }
    }
    int index = 0;
    int flag = -1;

    for (size_t i = 1; i < height -1; i++)
    {
      for (size_t j = 1; j < width -1; j++)
      {
        index = i * width + j;
        flag = flags[index];
        if (flag < 9999)
        {
          int key = (flags[index + width] == flag) * 4 + (flags[index + width + 1] == flag) * 2 + (flags[index + 1] == flag);
          switch (key)
          {
            case 7:
              cross_products[flag] += points_[index].cross(points_[index + width]) +
                  points_[index + width].cross(points_[index + width + 1]) +
                  points_[index + width + 1].cross(points_[index + 1]) +
                  points_[index + 1].cross(points_[index]);
              break;
            case 3:
              cross_products[flag] += points_[index].cross(points_[index + width + 1]) +
                  points_[index + width + 1].cross(points_[index + 1]) +
                  points_[index + 1].cross(points_[index]);
              break;
            case 5:
              cross_products[flag] += points_[index].cross(points_[index + width]) +
                  points_[index + width].cross(points_[index + 1]) +
                  points_[index + 1].cross(points_[index]);
              break;
            case 6:
              cross_products[flag] += points_[index].cross(points_[index + width]) +
                  points_[index + width].cross(points_[index + width + 1]) +
                  points_[index + width + 1].cross(points_[index]);
              break;
            default:
              break;
          }
          if ((flags[index - width - 1] != flag) && (flags[index - 1] == flag) && (flags[index - width] == flag))
          {
            cross_products[flag] += points_[index].cross(points_[index - width]) +
                points_[index - width].cross(points_[index - 1]) +
                points_[index - 1].cross(points_[index]);
          }
        }
      }
    }
    for (PlanarSegment::StdVector::iterator it = segments->begin (); it != segments->end(); it++)
    {
      it->area = 0.5 * fabs(it->normal.dot(cross_products[it - segments->begin ()]));
    }
    delete [] flags;

    //stop the timer
    if (verbose_)
    {
      gettimeofday(&tpend,NULL);
      timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
      timeuse/=1000000;
      area_calculation_time << " " << timeuse;
      area_calculation_time.close ();
      std::cerr << "area calcualtion time: " << timeuse << std::endl;
    }
  }

  void
  SegmentsArea::areaByDelaunayTriangulation (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments)
  {
    struct timeval tpstart,tpend;
    double timeuse;
    //start the timer
    if (verbose_)
    {
      gettimeofday(&tpstart,NULL);
    }

    std::vector<CGALPoint2> cgal_points;
    PlanarSegment::StdVector::iterator it;

    for (it = segments->begin (); it != segments->end (); it++)
    {
      double area = 0.0;
      projectSegmentTo2D (*it, cgal_points);
      Delaunay delaunay_triangulation;
      delaunay_triangulation.insert (cgal_points.begin (), cgal_points.end ());
      //Delaunay::All_faces_iterator fit;
      //for (fit = delaunay_triangulation.all_faces_begin (); fit != delaunay_triangulation.all_faces_end (); fit ++)
      Delaunay::Face_iterator fit;
      for (fit = delaunay_triangulation.faces_begin(); fit != delaunay_triangulation.faces_end (); fit ++)
      {
        CGALPoint2 aa = fit->vertex(0)->point();//cgal_points[fit->vertex(0)];
        CGALPoint2 bb = fit->vertex(1)->point();//cgal_points[fit->vertex(1)];
        CGALPoint2 cc = fit->vertex(2)->point();//cgal_points[fit->vertex(2)];
        area += fabs(aa.x () * bb.y () - aa.x () * cc.y () +
                     bb.x () * cc.y () - bb.x () * aa.y () +
                     cc.x () * aa.y () - cc.x () * bb.y ());
      }
      it->area = 0.5 * area;
    }

    //stop the timer
    if (verbose_)
    {
      gettimeofday(&tpend,NULL);
      timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
      timeuse/=1000000;
      std::cout << "Area calculation time using the DelaunayTriangulation method: " << timeuse << std::endl;
    }
  }

  void
  SegmentsArea::areaByAlphaShape (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments)
  {
    struct timeval tpstart,tpend;
    double timeuse;
    //start the timer
    //start the timer
    std::ofstream area_calculation_time;
    if (verbose_)
    {
      area_calculation_time.open("area_calculation_time.txt", std::fstream::app);
      gettimeofday(&tpstart,NULL);
    }

    std::vector<CGALPoint2> cgal_points;
    PlanarSegment::StdVector::iterator it;
    for (it = segments->begin (); it != segments->end (); it++)
    {
      //std::cerr << "segment " << it - segments->begin () << " with " << it->points.size() << " points.\n";
      double area = 0.0;
      Alpha_shape_2 alpha_shape;
      projectSegmentTo2D (*it, cgal_points);

//      char buf[4];
//      sprintf(buf, "%03d", it - segments->begin ());
//      std::string file_name;
//      file_name = std::string(buf) + ".txt";
//      std::ofstream of;
//      of.open (file_name.c_str ());
//      for (std::vector<CGALPoint2>::iterator pit = cgal_points.begin (); pit != cgal_points.end (); pit ++)
//      {
//        of << pit->x() << " " << pit->y() << std::endl;
//      }
//      of.close ();
      alpha_shape.set_mode(Alpha_shape_2::GENERAL);
      alpha_shape.make_alpha_shape(cgal_points.begin(), cgal_points.end());
      Alpha_shape_2::Alpha_iterator opt = alpha_shape.find_optimal_alpha (1);
//      std::cerr << "Optimum alpha value found: " << *opt << std::endl;
      alpha_shape.set_alpha(*opt);

      for (Alpha_shape_2::Finite_faces_iterator fit = alpha_shape.finite_faces_begin ();
           fit != alpha_shape.finite_faces_end (); fit ++)
      {
        if (alpha_shape.classify (fit) == Alpha_shape_2::INTERIOR)
        {
          CGALPoint2 aa = fit->vertex(0)->point();//cgal_points[fit->(0)];
          CGALPoint2 bb = fit->vertex(1)->point();//cgal_points[fit->ccw(1)];
          CGALPoint2 cc = fit->vertex(2)->point();//cgal_points[fit->ccw(2)];
          area += fabs(aa.x () * bb.y () - aa.x () * cc.y () +
                       bb.x () * cc.y () - bb.x () * aa.y () +
                       cc.x () * aa.y () - cc.x () * bb.y ());
        }
      }
      it->area = 0.5 * area;
    }

    //stop the timer
    if (verbose_)
    {
      gettimeofday(&tpend,NULL);
      timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
      timeuse/=1000000;
      area_calculation_time << " " << timeuse << std::endl;
      area_calculation_time.close ();
    }
  }


  void
  SegmentsArea::projectSegmentTo2D(PlanarSegment segment, std::vector<CGALPoint2> &cgal_points)
  {
    Eigen::EigenSolver<Eigen::Matrix3d> eigensolver;
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Zero();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Zero();
    int min_eigenvalue_index, max_eigenvalue_index;
    double min_eigenvalue, max_eigenvalue;

    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    Eigen::Matrix3d real_bases;


    eigensolver.compute(segment.scatter_matrix);
    eigenvalues = eigensolver.eigenvalues().real();
    eigenvectors = eigensolver.eigenvectors().real();
    min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
    max_eigenvalue = eigenvalues.maxCoeff(&max_eigenvalue_index);


    real_bases.col(2) = eigenvectors.col(min_eigenvalue_index);
    real_bases.col(0) = eigenvectors.col(max_eigenvalue_index);
    real_bases.col(1) = real_bases.col(2).cross(real_bases.col(0)).normalized();

    //std::cerr << real_bases.col(0).dot(real_bases.col(2)) << std::endl;

    rotation = real_bases.transpose();

    cgal_points.clear ();
    cgal_points.resize (segment.points.size ());

    Eigen::Vector3d tmp;
    size_t j = 0;
    for (std::vector<int>::iterator it = segment.points.begin (); it != segment.points.end (); it++, j++)
    {
      tmp = rotation * points_[*it];
      cgal_points[j] = CGALPoint2(tmp(0), tmp(1));

    }
  }


  void
  SegmentsArea::areaByNumberOfSquareUnits (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, PlanarSegment::StdVectorPtr segments)
  {
    struct timeval tpstart,tpend;
    double timeuse;
    //start the timer
    if (verbose_)
    {
      gettimeofday(&tpstart,NULL);
    }

    double tan_theta = tan(vertical_resolution_ * M_PI / 180);
    double tan_phi = tan(horizontal_resolution_ * M_PI / 180);

    Eigen::Vector3d point;
    for (PlanarSegment::StdVector::iterator it = segments->begin (); it != segments->end (); it++)
    {
      double area = 0.0;
      for (std::vector<int>::iterator sub_it = it->points.begin(); sub_it != it->points.end (); sub_it++)
      {
        point = points_[*sub_it];
        area += point.squaredNorm() * tan_theta * tan_phi / (it->normal.dot(point.normalized()));
      }
      it->area = area;
    }

    //stop the timer
    if (verbose_)
    {
      gettimeofday(&tpend,NULL);
      timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
      timeuse/=1000000;
      std::cout << "Area calculation time using the NumberOfSquareUnits method: " << timeuse << std::endl;
    }
  }

  void
  SegmentsArea::toEigenTypes (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    points_.clear ();
    points_.resize (cloud->size ());

    for (size_t i = 0; i < cloud->size (); i++)
    {
      points_[i](0) = cloud->points[i].x;
      points_[i](1) = cloud->points[i].y;
      points_[i](2) = cloud->points[i].z;
    }

  }
}
