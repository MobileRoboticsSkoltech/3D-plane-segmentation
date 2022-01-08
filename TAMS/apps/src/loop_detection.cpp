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

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//Eigen
#include <Eigen/Core>
//tams
#include "application_options_manager/application_options_manager.h"
#include "octree_region_growing_segmentation/octree_region_growing_segmentation.h"
#include "loop_detector/loop_detector.h"
#include "common/planar_patch.h"

using namespace tams;

int
main (int argc, char **argv)
{

  OctreeRGSegmentation segmenter;
  LoopDetector loop_detector;
  ApplicationOptionsManager amgr;

  PlanarSegment::StdVectorPtr segments(new PlanarSegment::StdVector);
  AbstractPlanarSegment::StdVectorPtr abstract_segments(new AbstractPlanarSegment::StdVector);
  AbstractPlanarSegment abstract_segment;

  if (!amgr.readOptions (argc, argv))
    return -1;

  pcl::PCDReader reader;
  pcl::visualization::PCLVisualizer *pViewer;
  pViewer = new pcl::visualization::PCLVisualizer (argc, argv, "illustrate loop detection related stuff");
  pViewer->setBackgroundColor (1,1,1);
  pViewer->spin ();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string prefix = amgr.app_options_.organized_pcd_dir + amgr.app_options_.input_prefix;

  enum DatasetType { BRL, ScrapYard, CrashedCarPark };
  DatasetType dataset = BRL;

  for (int scan_index = amgr.app_options_.first_index; scan_index <= amgr.app_options_.last_index; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = prefix + std::string (buf) + ".pcd";
    if (reader.read (pcd_file, *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("\nLoaded %d points with width: %d and height: %d from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str ());

    if (dataset == BRL)
    {
      for (int i = 0; i < cloud->size(); i++)
      {
        if (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y + cloud->points[i].z * cloud->points[i].z > 30 * 30)
        {
          cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = NAN;
        }
        if (cloud->points[i].z < 0 && (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y) < 0.7225)
        {
          cloud->points[i].x = cloud->points[i].y = cloud->points[i].z = NAN;
        }
      }
    }

    //segmentation
    segmenter.setParameters (amgr.octree_seg_params_);
    segmenter.setInput (cloud);
    segmenter.octreeCaching();
    segmenter.segmentation ();
    //area calculation
    //segmenter.segmentsArea();
    //abstract segments
    segments = segmenter.getSegments();
    abstract_segment.setSensorNoiseModel(amgr.sensor_params_.polynomial_noise_a0,
                                         amgr.sensor_params_.polynomial_noise_a1,
                                         amgr.sensor_params_.polynomial_noise_a2);
    abstract_segments->clear();
    for (PlanarSegment::StdVector::iterator it = segments->begin(); it != segments->end(); it++)
    {
      abstract_segment.calculateAttributes(it, cloud);
      abstract_segments->push_back(abstract_segment);
    }

    loop_detector.setSegments (abstract_segments);
    loop_detector.filterSegmentsByArea (0.5);
    loop_detector.sortSegmentsByArea ();
    loop_detector.enumerateDominantSegmentPair ();

  }
}
