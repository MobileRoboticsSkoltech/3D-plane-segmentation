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

//STL
#include <vector>
#include <fstream>
//PCL
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
//boost
#include <boost/timer.hpp>
//tams
#include "region_growing_segmentation/region_growing_segmentation.h"
#include "region_growing_segmentation/impl/region_growing_segmentation.hpp"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation.h"
#include "application_options_manager/application_options_manager.h"
#include "abstract_planar_segment/abstract_planar_segment.h"
#include "segments_area/segments_area.h"

using namespace tams;
using namespace Eigen;

int
main (int argc, char **argv)
{
  ApplicationOptionsManager amgr;
  if (!amgr.readOptions (argc, argv))
    return -1;
  OctreeRGSegmentation octree_segmenter;
  PlanarSegment::StdVectorPtr segments(new PlanarSegment::StdVector);
  AbstractPlanarSegment::StdVectorPtr abstract_segments(new AbstractPlanarSegment::StdVector);
  AbstractPlanarSegment abstract_segment;

  struct timeval tpstart,tpend;
  double timeuse;

  enum DatasetType { BRL, ScrapYard, CrashedCarPark, BremenCity, DellBoxes};
  DatasetType dataset = BremenCity;

  pcl::visualization::PCLVisualizer *pViewer;
  pViewer = new pcl::visualization::PCLVisualizer (argc, argv, "show area attributed planar segments");
  pViewer->setBackgroundColor (1,1,1);
  pViewer->spin ();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ognzd_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string prefix = amgr.app_options_.organized_pcd_dir + amgr.app_options_.input_prefix;
  std::ofstream time_output;
  time_output.open("k50.txt");
  for (int scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    std::string segments_file = amgr.app_options_.output_dir + "/segments" + std::string (buf);
    std::string screenshot = "/home/tams/workspace/pcl/segments/segment" + std::string (buf);
    std::string png_file;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *ognzd_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
              ognzd_cloud->points.size(), ognzd_cloud->width, ognzd_cloud->height, pcd_file.c_str ());

    if (dataset == BRL)
    {
      for (int i = 0; i < ognzd_cloud->size(); i++)
      {
        if (ognzd_cloud->points[i].x * ognzd_cloud->points[i].x +
            ognzd_cloud->points[i].y * ognzd_cloud->points[i].y +
            ognzd_cloud->points[i].z * ognzd_cloud->points[i].z > 30 * 30)

        {
          ognzd_cloud->points[i].x = ognzd_cloud->points[i].y = ognzd_cloud->points[i].z = NAN;
        }
        if (ognzd_cloud->points[i].z < 0 &&
            (ognzd_cloud->points[i].x * ognzd_cloud->points[i].x +
             ognzd_cloud->points[i].y * ognzd_cloud->points[i].y) < 0.7225)
        {
          ognzd_cloud->points[i].x = ognzd_cloud->points[i].y = ognzd_cloud->points[i].z = NAN;
        }
      }
    }

    octree_segmenter.setParameters (amgr.octree_seg_params_);
//    octree_segmenter.setSensorNoiseModel(amgr.sensor_params_.polynomial_noise_a0,
//                                         amgr.sensor_params_.polynomial_noise_a1,
//                                         amgr.sensor_params_.polynomial_noise_a2);
    octree_segmenter.setInput (ognzd_cloud);

    gettimeofday(&tpstart,NULL);
    octree_segmenter.octreeCaching();
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "Octree caching time: " << timeuse << std::endl;
    time_output << timeuse << " ";
    gettimeofday(&tpstart,NULL);
    octree_segmenter.segmentation ();
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "segmentation time: " << timeuse << std::endl;
    time_output << timeuse << " ";

    segments = octree_segmenter.getSegments();
    abstract_segment.setSensorNoiseModel(amgr.sensor_params_.polynomial_noise_a0,
                                         amgr.sensor_params_.polynomial_noise_a1,
                                         amgr.sensor_params_.polynomial_noise_a2);
    abstract_segments->clear();
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      abstract_segment.calculateAttributes(it, ognzd_cloud);
      abstract_segments->push_back(abstract_segment);
    }

    gettimeofday(&tpstart,NULL);
    octree_segmenter.uncertainties();
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "plane parameter uncertainty computation time: " << timeuse << std::endl;
    time_output << timeuse << std::endl;


//    gettimeofday(&tpstart,NULL);
//    octree_segmenter.segmentsArea();
//    gettimeofday(&tpend,NULL);
//    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
//    timeuse/=1000000;
//    std::cout << "area computation time: " << timeuse << std::endl;
//    time_output << timeuse << std::endl;

    std::vector<double> area;
    area.resize (4 * segments->size ());
    int i = 0;
    SegmentsArea areaByDelaunayTriangulation(ognzd_cloud, segments, SegmentsArea::DelaunayTriangulation);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaByAlphaShape(ognzd_cloud, segments, SegmentsArea::AlphaShape);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaByNumberOfSquareUnits(ognzd_cloud, segments, SegmentsArea::NumberOfSquareUnits, 0.5, 0.25);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaBySumOfSmallFaces(ognzd_cloud, segments, SegmentsArea::SumOfSmallFaces);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    std::string file_name;
    file_name = "scan" + std::string(buf) + "_area.txt";
    std::ofstream of;
    of.open (file_name.c_str ());
    for (size_t j = 0; j < segments->size (); j++)
    {
      of << area[j] << " "
         << area[j + segments->size ()] << " "
         << area[j + 2 * segments->size ()] << " "
         << area[j + 3 * segments->size ()] << std::endl;
      std::cerr << area[j] << " "
                << area[j + segments->size ()] << " "
                << area[j + 2 * segments->size ()] << " "
                << area[j + 3 * segments->size ()] << std::endl;
    }
    of.close ();


    if (amgr.app_options_.color_segments)
      octree_segmenter.randomColours (output, 0.0, true);
    //octree_segmenter.colourAccordingToUncertainty(output);

    segments = octree_segmenter.segments();

//    time_output << segments->size() << " " << timeuse << " ";
//    of.open(segments_file.c_str());
//    of << 0.0 << " " << 0.0 << " " << 0.0 << std::endl;
//    of << segments->size() << std::endl;
//    for (PlanarSegments::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
//    {
//      // normal.x normal.y normal.z bias mse area point_num
//      of << it->normal(0) << " " << it->normal(1) << " " << it->normal(2) << " " << it->bias << " "
//         << it->mse << " " << it->area << " " << it->point_num << " "
//         << it->mass_center(0) << " " << it->mass_center(1) << " " << it->mass_center(2) << std::endl;
//    }
//    of.close();

/*
    if (amgr.app_options_.color_segments)
    {
      pViewer->addPointCloud (output, "segments");
      pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segments");
      pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "segments");
      for (PlanarSegments::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
      {
        if (it->area < 0.5)
          continue;
        std::ostringstream area, area_full;
        area << setiosflags(ios::fixed) << setprecision(2) << it->area;
        //area << it->area;
        area_full << it->area;
        pcl::PointXYZ position;
        position.x = it->mass_center (0);
        position.y = it->mass_center (1);
        position.z = it->mass_center (2);
        pViewer->addText3D (area.str (), position, 0.4, 0.0, 0.0, 0.0, area_full.str (), 0);
      }
      //      pViewer->setCameraPose(50.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_0.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(0.0, 50.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_1.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(-50.0, 0.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_2.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(0.0, -50.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_3.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(0.0, -50.0, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_3.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(35.4, 35.4, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_4.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(-35.4, 35.4, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_5.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(-35.4, -35.4, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_6.png";
      //      pViewer->saveScreenshot(png_file);
      //      pViewer->setCameraPose(35.4, -35.4, 50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
      //      pViewer->spinOnce(100);
      //      png_file = screenshot + "_vp_7.png";
      //      pViewer->saveScreenshot(png_file);
      pViewer->spin();
      pViewer->removePointCloud("segments");
      for (PlanarSegments::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
      {
        if (it->area < 0.5)
          continue;
        std::ostringstream area, area_full;
        area << setiosflags(ios::fixed) << setprecision(2) << it->area;
        area_full << it->area;
        pViewer->removeText3D(area_full.str());
      }
      pViewer->spinOnce (100);
      //      pcl::visualization::PCLVisualizer viewer ("Detected planes with Pseudo-color");
      //      int viewports[2] = {1, 2};
      //      viewer.createViewPort (0.0, 0.0, 0.5, 1.0, viewports[0]);
      //      viewer.createViewPort (0.5, 0.0, 1.0, 1.0, viewports[1]);
      //      viewer.addPointCloud (output, "segments", viewports[0]);
      //      viewer.addPointCloud (output, "areaed_segments", viewports[1]);
      //      viewer.setBackgroundColor (1.0, 1.0, 1.0);
      //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "areaed_segments");
      //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "segments");
      //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "areaed_segments");
      //      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "segments");
      //      for (std::vector<tamrot::PlanarSegments>::iterator it = segments->begin(); it != segments->end(); it++)
      //      {
      //        if (it->area < 0.5)    std::vector<double> area;
    area.resize (4 * segments->size ());
    int i = 0;
    SegmentsArea areaByDelaunayTriangulation(ognzd_cloud, segments, SegmentsArea::DelaunayTriangulation);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaByAlphaShape(ognzd_cloud, segments, SegmentsArea::AlphaShape);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaByNumberOfSquareUnits(ognzd_cloud, segments, SegmentsArea::NumberOfSquareUnits, 0.5, 0.25);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    SegmentsArea areaBySumOfSmallFaces(ognzd_cloud, segments, SegmentsArea::SumOfSmallFaces);
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      area[i] = it->area;
      i++;
    }
    std::string file_name;
    file_name = "scan" + std::string(buf) + "_area.txt";
    std::ofstream of;
    of.open (file_name.c_str ());
    for (size_t j = 0; j < segments->size (); j++)
    {
      of << area[j] << " "
         << area[j + segments->size ()] << " "
         << area[j + 2 * segments->size ()] << " "
         << area[j + 3 * segments->size ()] << std::endl;
      std::cerr << area[j] << " "
                << area[j + segments->size ()] << " "
                << area[j + 2 * segments->size ()] << " "
                << area[j + 3 * segments->size ()] << std::endl;
    }
    of.close ();
      //          continue;
      //        std::ostringstream area;
      //        area << setiosflags(ios::fixed) <<setprecision(2) << it->area;
      //        pcl::PointXYZ position;
      //        position.x = it->mass_center (0);
      //        position.y = it->mass_center (1);
      //        position.z = it->mass_center (2);
      //        viewer.addText3D (area.str (), position, 0.6, 0.0, 0.0, 0.0, area.str (), viewports[1]);
      //      }
      //      viewer.spin ();
    }
*/
//    if (amgr.app_options_.color_segments)
//    {
//      pViewer->addPointCloud (output, "segments");
//      pViewer->setPointCloudRenderingProperties    std::vector<double> area;
//      pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "segments");
//      for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
//      {
//        if (it->area < 0.5)
//          continue;
//        std::ostringstream area, area_full;
//        area << setiosflags(ios::fixed) << setprecision(2) << it->area;
//        area_full << it->area;
//        pcl::PointXYZ position;
//        position.x = it->mass_center (0);
//        position.y = it->mass_center (1);
//        position.z = it->mass_center (2);
//        pViewer->addText3D (area.str (), position, 0.4, 0.0, 0.0, 0.0, area_full.str (), 0);
//      }
//      pViewer->spin();
//      pViewer->removePointCloud("segments");
//      for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
//      {
//        if (it->area < 0.5)
//          continue;
//        std::ostringstream area_full;
//        area_full << it->area;
//        pViewer->removeText3D(area_full.str());
//      }
//      pViewer->spinOnce (100);
//    }

    bool show_normal = false;
    bool show_area = true;
    if (amgr.app_options_.color_segments)
    {
      pViewer->addPointCloud (output, "segments");
      pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segments");
      pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "segments");
      for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
      {
        if (it->area < 0.2)
          continue;
        std::ostringstream area_full;
        area_full << it->area;
        if (show_normal)
        {
          pcl::PointXYZ start, end;
          start.x = it->mass_center (0);
          start.y = it->mass_center (1);
          start.z = it->mass_center (2);
          end.x = it->mass_center (0) - it->normal (0);
          end.y = it->mass_center (1) - it->normal (1);
          end.z = it->mass_center (2) - it->normal (2);
          pViewer->addArrow(end, start, 0.0, 0.0, 0.0, false, area_full.str ());
        }
        if (show_area)
        {
          std::ostringstream area;
          area << setiosflags(ios::fixed) << setprecision(2) << it->area;
          //area << it - segments->begin ();
          pcl::PointXYZ position;
          position.x = it->mass_center (0);
          position.y = it->mass_center (1);
          position.z = it->mass_center (2);
          pViewer->addText3D (area.str (), position, 0.2, 0.0, 0.0, 0.0, area_full.str (), 0);
        }
      }
      pViewer->spin();
      pViewer->removePointCloud("segments");
      if (show_normal)
      {
        pViewer->removeAllShapes ();
      }
      if (show_area)
      {
        for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
        {
          if (it->area < 0.2)
            continue;
          std::ostringstream area_full;
          area_full << it->area;
          pViewer->removeText3D(area_full.str());
        }
      }
      pViewer->spinOnce (100);
    }

  }
  delete pViewer;
  return (0);
}
