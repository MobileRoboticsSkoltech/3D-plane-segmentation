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
#include "common/common.h"
#include "abstract_planar_segment/abstract_planar_segment.h";
#include "segments_area/segments_area.h"
#include "registration/registration.h"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation.h"
#include "application_options_manager/application_options_manager.h"


void
transform(pcl::PointCloud<pcl::PointXYZ> &output,
          pcl::PointCloud<pcl::PointXYZ> &input,
          Eigen::Matrix3d rotation,
          Eigen::Vector3d translation);

int
main (int argc, char** argv)
{
  using namespace tams;
  using namespace Eigen;

  ApplicationOptionsManager amgr;
  if (!amgr.readOptions (argc, argv))
    return -1;
  OctreeRGSegmentation map_segmenter;
  OctreeRGSegmentation data_segmenter;

  AbstractPlanarSegment::StdVectorPtr abstract_segments(new AbstractPlanarSegment::StdVector);
  AbstractPlanarSegment abstract_segment;

  Registration registration;
  struct timeval tpstart,tpend;
  double timeuse;

  enum DatasetType { BRL, ScrapYard, CrashedCarPark, BremenCity};
  DatasetType dataset = BremenCity;

  std::vector<Vector3d, aligned_allocator<Vector3d> > successive_translations;
  std::vector<Matrix3d, aligned_allocator<Matrix3d> > successive_rotations;
  std::vector<Vector3d, aligned_allocator<Vector3d> > accumulate_translations;
  std::vector<Matrix3d, aligned_allocator<Matrix3d> > accumulate_rotations;
  Matrix3d rotation;
  Vector3d translation;

  pcl::PointCloud<pcl::PointXYZ> map;
  pcl::PointCloud<pcl::PointXYZ> output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  PlanarSegment::StdVectorPtr map_segments(new PlanarSegment::StdVector);
  PlanarSegment::StdVectorPtr data_segments(new PlanarSegment::StdVector);

  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_data_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string prefix = amgr.app_options_.organized_pcd_dir + amgr.app_options_.input_prefix;
  map_segmenter.setParameters (amgr.octree_seg_params_);
  data_segmenter.setParameters(amgr.octree_seg_params_);
  registration.setParameters(amgr.registration_params_);
  abstract_segment.setSensorNoiseModel(amgr.sensor_params_.polynomial_noise_a0,
                                       amgr.sensor_params_.polynomial_noise_a1,
                                       amgr.sensor_params_.polynomial_noise_a2);

  //the first point cloud
  char buf[4];
  sprintf (buf, "%03d", amgr.app_options_.first_index);
  std::string pcd_file = prefix + std::string (buf) + ".pcd";
  std::string pose_file = prefix + std::string (buf) + ".pose";
  std::string segments_file = amgr.app_options_.output_dir + "/segments" + std::string (buf);
  std::ofstream pose;
  std::ofstream parameter_turning;
  parameter_turning.open ("parameter_turning.txt");
  pcl::PCDReader pcd_reader;

  if (pcd_reader.read (pcd_file, *map_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
            map_cloud->points.size(), map_cloud->width, map_cloud->height, pcd_file.c_str ());

  //segmentation
  map_segmenter.setInput (map_cloud);
  gettimeofday(&tpstart,NULL);
  map_segmenter.octreeCaching();
  map_segmenter.segmentation();
  gettimeofday(&tpend,NULL);
  timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
  timeuse/=1000000;
  std::cerr << "segmentation time: " << timeuse << std::endl;
  for (PlanarSegment::StdVector::iterator it = map_segmenter.getSegments ()->begin(); it != map_segmenter.getSegments ()->end(); it++)
  {
    abstract_segment.calculateAttributes(it, map_cloud);
    it->normal = abstract_segment.normal;
    it->bias = abstract_segment.d;
  }

  SegmentsArea mapSegmentsArea (map_cloud, map_segmenter.getSegments (), SegmentsArea::SumOfSmallFaces);
  map_segments->clear();
  map_segments->insert(map_segments->begin(), map_segmenter.getSegments()->begin(), map_segmenter.getSegments()->end());

  pose.open (pose_file.c_str ());
  pose << 1.0f << " " << 0.0f << " " << 0.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 1.0f << " " << 0.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 0.0f << " " << 1.0f << " " << 0.0f << std::endl
       << 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << std::endl;
  pose.close ();

  registration.setMapCloud(map_cloud);
  registration.setMapSegments(map_segments);
  accumulate_translations.push_back(Vector3d::Zero());
  accumulate_rotations.push_back(Matrix3d::Identity());
  double path_length = 0.0;
  for (int scan_index = amgr.app_options_.first_index + 1; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    sprintf (buf, "%03d", scan_index);
    pcd_file = prefix + std::string (buf) + ".pcd";
    pose_file = prefix + std::string (buf) + ".pose";
    if (pcd_reader.read (pcd_file, *data_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points (width: %d and height: %d) from %s.\n",
              data_cloud->points.size(), data_cloud->width, data_cloud->height, pcd_file.c_str ());

    data_segmenter.setInput (data_cloud);
    gettimeofday(&tpstart,NULL);
    data_segmenter.octreeCaching();
    data_segmenter.segmentation();
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cerr << "segmentation time: " << timeuse << std::endl;

    for (PlanarSegment::StdVector::iterator it = data_segmenter.getSegments ()->begin(); it != data_segmenter.getSegments ()->end(); it++)
    {
      abstract_segment.calculateAttributes(it, data_cloud);
      it->normal = abstract_segment.normal;
      it->bias = abstract_segment.d;
    }
    SegmentsArea dataSegmentsArea (data_cloud, data_segmenter.getSegments (), SegmentsArea::SumOfSmallFaces);
    data_segments->clear();
    data_segments->insert(data_segments->begin(), data_segmenter.getSegments()->begin(), data_segmenter.getSegments()->end());

    registration.setDataCloud(data_cloud);
    registration.setDataSegments(data_segments);
    gettimeofday(&tpstart,NULL);
    registration.execute();
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "length of the step: " << (registration.translation()).norm() << std::endl;
    std::cout << "Registration time: " << timeuse << std::endl;

    double roll, pitch, yaw;
    gerRPYFromRotationMatrix(registration.rotation (), roll, pitch, yaw);
    //std::cerr << "step rotation: " << registration.rotation () << std::endl;
    std::cerr << "step translation: " <<
              registration.translation ()(0) << " & " <<
              registration.translation ()(1) << " & " <<
              registration.translation ()(2) << " & " << std::endl;
    std::cerr << "step RPY: " << roll * 180 / M_PI << " & " << pitch * 180 / M_PI << " & " << yaw * 180 / M_PI << std::endl;

    successive_rotations.push_back(registration.rotation());
    successive_translations.push_back(registration.translation());
    rotation = accumulate_rotations.back() * registration.rotation();
    translation = accumulate_rotations.back() * registration.translation() + accumulate_translations.back();
    //translation = registration.rotation() * accumulate_translations.back() + registration.translation();
    std::cout << "current position: " << translation.transpose() << std::endl;
    std::cout << "current orientation: " << "todo" << std::endl;
    accumulate_rotations.push_back(rotation);
    accumulate_translations.push_back(translation);

    pose.open (pose_file.c_str ());
    pose << rotation(0,0) << " " << " " << rotation(0,1) << " " << rotation(0,2) << " " << translation(0) << std::endl
         << rotation(1,0) << " " << " " << rotation(1,1) << " " << rotation(1,2) << " " << translation(1) << std::endl
         << rotation(2,0) << " " << " " << rotation(2,1) << " " << rotation(2,2) << " " << translation(2) << std::endl
         << 0.0f          << " " << " " <<          0.0f << " " <<          0.0f << " " <<           1.0f << std::endl;
    pose.close ();

    if (amgr.registration_params_.visualization)
    {
      //registration.visualizeCorrespondences();
      registration.visualizeCorrespondencesWithPoints ();
    }

    path_length += registration.translation().norm();
    *map_cloud = *data_cloud;
    map_segments->clear();
    map_segments->insert(map_segments->begin(), data_segments->begin(), data_segments->end());
    registration.setMapCloud(map_cloud);
    registration.setMapSegments(map_segments);
  }
  std::cout << "length of the path: " << path_length << std::endl;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    sprintf (buf, "%03d", scan_index);
    pcd_file = prefix + std::string (buf) + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(cloud,cloud,indices);
    transform(output, cloud,
              accumulate_rotations[scan_index - amgr.app_options_.first_index],
              accumulate_translations[scan_index - amgr.app_options_.first_index]);
    map.points.insert(map.points.begin(), output.points.begin(), output.points.end());
    map.height = 1;
    map.width = map.points.size();
  }

  pcl::PointCloud<pcl::PointXYZ> downsampled;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_downsampling;
  voxel_grid_downsampling.setInputCloud (map.makeShared());
  voxel_grid_downsampling.setLeafSize (0.1, 0.1, 0.1);
  voxel_grid_downsampling.filter (downsampled);
  std::cout << "cloud size after down-sampling: " << downsampled.size () << std::endl;

  pcl::io::savePCDFileBinary("map.pcd", map);
  pcl::io::savePCDFileBinary("downsampled_map.pcd", downsampled);
  return 0;
}

void transform(pcl::PointCloud<pcl::PointXYZ> &output,
               pcl::PointCloud<pcl::PointXYZ> &input,
               Eigen::Matrix3d rotation,
               Eigen::Vector3d translation)
{
  double rm[9];
  rm[0] = rotation(0,0);
  rm[1] = rotation(0,1);
  rm[2] = rotation(0,2);
  rm[3] = rotation(1,0);
  rm[4] = rotation(1,1);
  rm[5] = rotation(1,2);
  rm[6] = rotation(2,0);
  rm[7] = rotation(2,1);
  rm[8] = rotation(2,2);
  output.width = input.width;
  output.height = 1;
  output.points.resize(input.size());
  int cnt = 0;
  for (size_t i = 0; i < input.size(); i++)
  {
    if (input.points[i].x != 0 || input.points[i].y != 0 || input.points[i].z != 0)
    {
      output.points[cnt].x = rm[0] * input.points[i].x + rm[1] * input.points[i].y + rm[2] * input.points[i].z + translation(0);
      output.points[cnt].y = rm[3] * input.points[i].x + rm[4] * input.points[i].y + rm[5] * input.points[i].z + translation(1);
      output.points[cnt].z = rm[6] * input.points[i].x + rm[7] * input.points[i].y + rm[8] * input.points[i].z + translation(2);
      cnt ++;
    }
  }
  output.points.erase(output.points.begin()+cnt, output.points.end());
  output.width = cnt;
  output.height = 1;
}

