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
#include "common/common.h"

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

  struct timeval tpstart,tpend;
  double timeuse;

  pcl::visualization::PCLVisualizer *pViewer;
  pViewer = new pcl::visualization::PCLVisualizer (argc, argv, "show area attributed planar segments");
  pViewer->setBackgroundColor (1,1,1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  std::string pcd_file = amgr.app_options_.pcd_file;
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
    return (-1);
  }
  PCL_INFO ("Loaded %d points with width: %d and height: %d from %s.\n",
            cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str ());

  octree_segmenter.setParameters (amgr.octree_seg_params_);
  octree_segmenter.setInput (cloud);

  gettimeofday(&tpstart,NULL);
  octree_segmenter.octreeCaching();
  gettimeofday(&tpend,NULL);
  timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
  timeuse/=1000000;
  std::cout << "Octree caching time: " << timeuse << std::endl;
  gettimeofday(&tpstart,NULL);
  octree_segmenter.segmentation ();
  gettimeofday(&tpend,NULL);
  timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
  timeuse/=1000000;
  std::cout << "segmentation time: " << timeuse << std::endl;

  if (amgr.app_options_.color_segments)
  {
    segments = octree_segmenter.getSegments ();
    std::cerr << "enter alpha shape area computation.\n";
    SegmentsArea areaByAlphaShape(cloud, segments, SegmentsArea::AlphaShape);
    std::cerr << "area of each segment has been computed, enter segment randomly colouring.\n";
    randomColours(cloud, output, segments, 1.0, true);
    std::cerr << "segments have been coloured randomly.\n";
    pViewer->addPointCloud (output, "segments");
    pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "segments");
    pViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, "segments");
    pViewer->spin();
  }

  delete pViewer;
  return (0);
}

