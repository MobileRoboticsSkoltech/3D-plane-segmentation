// Copyright (c) 2003,2006  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Apollonius_graph_2/include/CGAL/Apollonius_graph_2/Traits_wrapper_2.h $
// $Id: Traits_wrapper_2.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Menelaos Karavelas <mkaravel@iacm.forth.gr>



#ifndef CGAL_APOLLONIUS_GRAPH_2_TRAITS_WRAPPER_2_H
#define CGAL_APOLLONIUS_GRAPH_2_TRAITS_WRAPPER_2_H

#include <CGAL/license/Apollonius_graph_2.h>



#include <CGAL/Apollonius_graph_2/basic.h>



namespace CGAL {

namespace ApolloniusGraph_2 {


template<class Gt_base>
class Apollonius_graph_traits_wrapper_2 : public Gt_base
{
public:
  //  struct Segment_2  {};
  struct Triangle_2 {};

  Apollonius_graph_traits_wrapper_2() {}
  Apollonius_graph_traits_wrapper_2(const Gt_base& gtb)
    : Gt_base(gtb) {}

};


} //namespace ApolloniusGraph_2

} //namespace CGAL


#endif // CGAL_APOLLONIUS_GRAPH_2_TRAITS_WRAPPER_2_H
