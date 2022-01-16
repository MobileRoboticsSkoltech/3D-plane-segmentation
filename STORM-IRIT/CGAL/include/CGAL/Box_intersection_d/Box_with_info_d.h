// Copyright (c) 2004  Max-Planck-Institute Saarbruecken (Germany).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Box_intersection_d/include/CGAL/Box_intersection_d/Box_with_info_d.h $
// $Id: Box_with_info_d.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Lutz Kettner  <kettner@mpi-sb.mpg.de>
//                 Andreas Meyer <ameyer@mpi-sb.mpg.de>

#ifndef CGAL_BOX_INTERSECTION_D_BOX_WITH_INFO_D_H
#define CGAL_BOX_INTERSECTION_D_BOX_WITH_INFO_D_H

#include <CGAL/license/Box_intersection_d.h>


#include <CGAL/basic.h>
#include <CGAL/Box_intersection_d/Box_d.h>

namespace CGAL {

namespace Box_intersection_d {

template<class NT_, int N, class Info_> 
class Box_with_info_d : public Box_d< NT_, N, ID_FROM_BOX_ADDRESS> {
protected:
    Info_ m_info;
public:
    typedef Box_d< NT_, N, ID_FROM_BOX_ADDRESS> Base;
    typedef NT_                      NT;
    typedef Info_                     Info;
    Box_with_info_d() {}
    Box_with_info_d( Info h) : m_info(h) {}
    Box_with_info_d( bool complete, Info h): Base(complete), m_info(h) {}
    Box_with_info_d(NT l[N], NT h[N], Info n) : Base( l, h), m_info(n) {}
    Box_with_info_d( const Bbox_2& b, Info h) : Base( b), m_info(h) {}
    Box_with_info_d( const Bbox_3& b, Info h) : Base( b), m_info(h) {}
    Info info() const { return m_info; }
};

} // end namespace Box_intersection_d


} //namespace CGAL

#endif
