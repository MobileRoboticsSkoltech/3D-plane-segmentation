// Copyright (c) 2008 ETH Zurich (Switzerland)
// Copyright (c) 2008-2009  INRIA Sophia-Antipolis (France) 
// Copyright (c) 2009  GeometryFactory (France)
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Intersections_3/include/CGAL/Intersections_3/internal/Bbox_3_Bbox_3_do_intersect.h $
// $Id: Bbox_3_Bbox_3_do_intersect.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     :  Laurent Rineau, Camille Wormser, Jane Tournois, Pierre Alliez

#ifndef CGAL_INTERNAL_INTERSECTIONS_3_BBOX_3_BBOX_3_DO_INTERSECT_H
#define CGAL_INTERNAL_INTERSECTIONS_3_BBOX_3_BBOX_3_DO_INTERSECT_H

// Turn off Visual C++ warning
#ifdef _MSC_VER
#pragma warning ( disable : 4003 )
#endif

#include <CGAL/Bbox_3.h>

namespace CGAL {
  bool
  inline
  do_intersect(const CGAL::Bbox_3& c,
               const CGAL::Bbox_3& bbox)
  {
    return CGAL::do_overlap(c, bbox);
  }

} //namespace CGAL

#endif  // CGAL_INTERNAL_INTERSECTIONS_3_BBOX_3_BBOX_3_DO_INTERSECT_H
