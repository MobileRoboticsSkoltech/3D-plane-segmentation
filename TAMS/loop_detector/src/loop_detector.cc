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

#include "loop_detector/loop_detector.h"
#include "planar_segments_matcher/one_pair_tester.h"

namespace tams
{
  LoopDetector::LoopDetector ():
    segments_ (new AbstractPlanarSegment::StdVector), sorted_segments_ (),retained_segments_ ()
  {

  }

  LoopDetector::LoopDetector (AbstractPlanarSegment::StdVectorPtr segments):
    segments_ (segments), sorted_segments_ (),retained_segments_ ()
  {
  }

  void
  LoopDetector::setSegments(AbstractPlanarSegment::StdVectorPtr segments)
  {
    segments_ = segments;
  }

  LoopDetector::~LoopDetector()
  {

  }
  /////////////////////////////////////////////////////////////////////////
  void
  LoopDetector::sortSegmentsByArea()
  {
    sorted_segments_.clear();
    sorted_segments_ = retained_segments_;

    AbstractPlanarSegment::StdVector::iterator it1;
    AbstractPlanarSegment::StdVector::iterator it2;
    AbstractPlanarSegment::Ptr pSegment;
    for (size_t i = 0; i < sorted_segments_.size () - 1; i++)
    {
      for (size_t j = i + 1; j < sorted_segments_.size (); j++)
      {
        if (sorted_segments_[i]->area < sorted_segments_[j]->area)
        {
          pSegment = sorted_segments_[j];
          sorted_segments_[j] = sorted_segments_[i];
          sorted_segments_[i] = pSegment;
        }
      }
    }
//    for (it1 = sorted_segments_.begin (); it1 != sorted_segments_.end () - 1; it1++)
//    {
//      for (it2 = it1 + 1; it2 < sorted_segments_->end (); it2++)
//      {
//        if (it2->area > it1->area)
//        {
//          segment = *it2;
//          *it2 = *it1;
//          *it1 = segment;
//        }
//      }
//    }

//    for (size_t i = 0; i < sorted_segments_.size (); i++)
//    {
//      std::cerr << sorted_segments_[i]->area << std::endl;
//    }
    std::cerr << "the segments have been sorted according to their area.\n";
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  LoopDetector::filterSegmentsByArea(double alpha)
  {
    retained_segments_.clear ();
    AbstractPlanarSegment::StdVector::iterator it;
    for (it = segments_->begin (); it != segments_->end (); it++)
    {
      if (it->area > alpha)
      {
        AbstractPlanarSegment::Ptr pSegment(new AbstractPlanarSegment);
        *pSegment = *it;
        retained_segments_.push_back (pSegment);
      }
    }
    std::cerr << "there are " << retained_segments_.size () << " planar segments whose area is bigger than " << alpha << "." << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  void
  LoopDetector::enumerateDominantSegmentPair()
  {
    std::vector<AbstractPlanarSegment::Ptr> dominant_segments;
    dominant_segments.clear ();
    dominant_segments.push_back (sorted_segments_[0]);

    size_t i = 1;
    while (i < sorted_segments_.size () && dominant_segments.size () < 2)
    {
      if (!areOnSameInfinitePlane(dominant_segments[0], sorted_segments_[i], 0.996, 0.05))
      {
        dominant_segments.push_back (sorted_segments_[i]);
      }
      i++;
    }

    if (i == sorted_segments_.size ())
    {
      std::cerr << "All segments are on the same infinite plane, something worong happened!\n";
    }

    while (i < sorted_segments_.size () && dominant_segments.size () < 3)
    {
      if (!areOnSameInfinitePlane(dominant_segments[0], sorted_segments_[i], 0.996, 0.05) &&
          !areOnSameInfinitePlane(dominant_segments[1], sorted_segments_[i], 0.996, 0.05))
      {
        dominant_segments.push_back (sorted_segments_[i]);
      }
      i++;
    }

    if (i == sorted_segments_.size ())
    {
      std::cerr << "Only two segments which are not on the same infinite plane have been found!\n";
    }
    else
    {
      std::cerr << "Three segments which are not on the same infinite plane have been found!\n";
    }

  }

  bool
  LoopDetector::areOnSameInfinitePlane(AbstractPlanarSegment::Ptr pSegment_1,
                                       AbstractPlanarSegment::Ptr pSegment_2,
                                       double minimum_dot_product,
                                       double maximum_distance)
  {
    bool ret = false;
    double dot_product = pSegment_1->normal.dot(pSegment_2->normal);
    double distance = fabs(pSegment_1->d - pSegment_2->d);
//    std::cerr << "dot product: " << dot_product << " ; distance: " << distance << std::endl;

    if (dot_product > minimum_dot_product && distance < maximum_distance)
    {
      ret = true;
    }

    return ret;
  }
}
