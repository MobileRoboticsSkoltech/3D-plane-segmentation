// Copyright (c) 1997-2001  
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved. 
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Optimisation_basic/include/CGAL/Optimisation/Access_dimension_2.h $
// $Id: Access_dimension_2.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Sven Schoenherr <sven@inf.ethz.ch>

#ifndef CGAL_OPTIMISATION_ACCESS_DIMENSION_2_H
#define CGAL_OPTIMISATION_ACCESS_DIMENSION_2_H



namespace CGAL {

// Class declaration
// =================
template < class R_ >
class Access_dimension_2;

// Class interface
// ===============
template < class R_ >
class Access_dimension_2 {
  public:
    // self
    typedef  R_                         R;
    typedef  Access_dimension_2<R>      Self;

    // types
    typedef  typename R::Point_2        Point;

    // unary function class types
    typedef  int                        result_type;
    typedef  Point                      argument_type;

    // creation
    Access_dimension_2( ) { }

    // operations
    int  operator() ( const Point& p) const { return p.dimension(); }
};

} //namespace CGAL

#endif // CGAL_OPTIMISATION_ACCESS_DIMENSION_2_H

// ===== EOF ==================================================================
