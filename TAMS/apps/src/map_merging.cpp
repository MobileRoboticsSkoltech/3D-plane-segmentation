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
#include <string>
#include <cmath>
#include <fstream>
//PCl
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <boost/timer.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//TAMS
#include "abstract_planar_segment/abstract_planar_segment.h";
#include "segments_area/segments_area.h"
#include "registration/registration.h"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation.h"
#include "application_options_manager/application_options_manager.h"

int
main (int argc, char** argv)
{
  using namespace tams;
  using namespace Eigen;

  ApplicationOptionsManager amgr;
  if (!amgr.readOptions (argc, argv))
    return -1;

  OctreeRGSegmentation segmenter1;
  OctreeRGSegmentation segmenter2;

  AbstractPlanarSegment::StdVectorPtr abstract_segments(new AbstractPlanarSegment::StdVector);
  AbstractPlanarSegment abstract_segment;

  Registration registration;
  struct timeval tpstart,tpend;
  double timeuse;

  Matrix3d rotation;
  Vector3d translation;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
  PlanarSegment::StdVectorPtr segments1(new PlanarSegment::StdVector);
  PlanarSegment::StdVectorPtr segments2(new PlanarSegment::StdVector);


  segmenter1.setParameters (amgr.octree_seg_params_);
  segmenter2.setParameters (amgr.octree_seg_params_);
  registration.setParameters(amgr.registration_params_);
  abstract_segment.setSensorNoiseModel(amgr.sensor_params_.polynomial_noise_a0,
                                       amgr.sensor_params_.polynomial_noise_a1,
                                       amgr.sensor_params_.polynomial_noise_a2);

  //the first map
  std::string pcd_file1 = "/media/work/datasets/BremenCity/map/map1.pcd";
  std::string pose_file1 = "/media/work/datasets/BremenCity/map/map1.pose";
  //the second map
  std::string pcd_file2 = "/media/work/datasets/BremenCity/map/map2.pcd";
  std::string pose_file2 = "/media/work/datasets/BremenCity/map/map2.pose";

  std::ofstream pose;
  pcl::PCDReader pcd_reader;

  if (pcd_reader.read (pcd_file1, *cloud1) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file1.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
            cloud1->points.size(), cloud1->width, cloud1->height, pcd_file1.c_str ());

  //segmentation
  segmenter1.setInput (cloud1);
  segmenter1.octreeCaching();
  segmenter1.segmentation();
  for (PlanarSegment::StdVector::iterator it = segmenter1.getSegments ()->begin(); it != segmenter1.getSegments ()->end(); it++)
  {
    abstract_segment.calculateAttributes(it, segmenter1.getCloud ());
    it->normal = abstract_segment.normal;
    it->bias = abstract_segment.d;
  }
  SegmentsArea segment_area1 (cloud1, segmenter1.getSegments (), SegmentsArea::AlphaShape);
  segments1->clear();
  segments1->insert(segments1->begin(), segmenter1.getSegments()->begin(), segmenter1.getSegments()->end());

  pose.open (pose_file1.c_str ());
  pose << 1.0f << " " << 0.0f << " " << 0.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 1.0f << " " << 0.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 0.0f << " " << 1.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << std::endl;
  pose.close ();

  registration.setMapCloud(cloud1);
  registration.setMapSegments(segments1);

  if (pcd_reader.read (pcd_file2, *cloud2) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file2.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
            cloud2->points.size(), cloud2->width, cloud2->height, pcd_file2.c_str ());

  //segmentation
  segmenter2.setInput (cloud2);
  segmenter2.octreeCaching();
  segmenter2.segmentation();
  for (PlanarSegment::StdVector::iterator it = segmenter2.getSegments ()->begin();
       it != segmenter2.getSegments ()->end(); it++)
  {
    abstract_segment.calculateAttributes(it, segmenter2.getCloud ());
    it->normal = abstract_segment.normal;
    it->bias = abstract_segment.d;
  }
  SegmentsArea segment_area2 (cloud2, segmenter2.getSegments (), SegmentsArea::AlphaShape);
  segments2->clear();
  segments2->insert(segments2->begin(), segmenter2.getSegments()->begin(), segmenter2.getSegments()->end());

  registration.setDataCloud(cloud2);
  registration.setDataSegments(segments2);

  gettimeofday(&tpstart,NULL);
  registration.execute();
  gettimeofday(&tpend,NULL);
  timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
  timeuse/=1000000;
  std::cout << "length of the step: " << (registration.translation()).norm() << std::endl;
  std::cout << "Registration time: " << timeuse << std::endl;

  rotation = registration.rotation ();
  translation = registration.translation ();

  pose.open (pose_file2.c_str ());
  pose << rotation(0,0) << " " << " " << rotation(0,1) << " " << rotation(0,2) << " " << translation(0) << std::endl
       << rotation(1,0) << " " << " " << rotation(1,1) << " " << rotation(1,2) << " " << translation(1) << std::endl
       << rotation(2,0) << " " << " " << rotation(2,1) << " " << rotation(2,2) << " " << translation(2) << std::endl
       << 0.0f          << " " << " " <<          0.0f << " " <<          0.0f << " " <<           1.0f << std::endl;
  pose.close ();

  if (amgr.registration_params_.visualization)
    registration.visualizeCorrespondences();

  return 0;
}

