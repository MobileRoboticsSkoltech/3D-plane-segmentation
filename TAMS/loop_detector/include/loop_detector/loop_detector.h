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

#ifndef LOOP_DETECTOR_H_
#define LOOP_DETECTOR_H_

//STL
#include <algorithm>
#include <vector>
//Eigen
#include "Eigen/Core"
#include "Eigen/Geometry"
//PCL
#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
//tams
#include "abstract_planar_segment/abstract_planar_segment.h"

namespace tams
{
  class LoopDetector
  {
      /** Typedefs.*/

    public:
      /** \brief Empty constructor. */
      LoopDetector();

      /** \brief Constructor with segments. */
      LoopDetector(AbstractPlanarSegment::StdVectorPtr segment);

      /** \brief Set segments. */
      void setSegments(AbstractPlanarSegment::StdVectorPtr segment);

      /** \brief Empty deconstructor. */
      ~LoopDetector();

      /** \brief Try to find (a) dominant segment pair, where the two
        segments have large area and they are non-parallel. */
      void
      enumerateDominantSegmentPair();

      /** \brief Filter out segments whose area is smaller than a pre-defined threshold.*/
      void
      filterSegmentsByArea(double alpha);

      /** \brief Sort the segments according to their area.*/
      void
      sortSegmentsByArea();

      /** \brief Test whether two segments are on the same infinite plane.
        * Two segments are considered to be on the same infinite plane if n1.dot(n2) > minimum_dot_product and abs(d1 - d2) < maximum_distance.
        * \param pSegment_1 boost shared pointer which points to one abstract planar segment
        * \param pSegment_2 boost shared pointer which points to another abstract planar segment
      */
      bool
      areOnSameInfinitePlane(AbstractPlanarSegment::Ptr pSegment_1,
                             AbstractPlanarSegment::Ptr pSegment_2,
                             double minimum_dot_product,
                             double maximum_distance);

    private:
      AbstractPlanarSegment::StdVectorPtr segments_;
      std::vector<AbstractPlanarSegment::Ptr> sorted_segments_;
      std::vector<AbstractPlanarSegment::Ptr> retained_segments_;
      //AbstractPlanarSegment::StdVectorPtr retained_segments_;
  };
}
#endif
