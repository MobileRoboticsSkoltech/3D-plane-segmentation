// Copyright (c) 2010 GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Mesh_3/include/CGAL/Mesh_3/Detect_polylines_in_polyhedra_fwd.h $
// $Id: Detect_polylines_in_polyhedra_fwd.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent Rineau
//

#ifndef CGAL_DETECT_POLYLINES_IN_POLYHEDRA_FWD_H
#define CGAL_DETECT_POLYLINES_IN_POLYHEDRA_FWD_H

#include <CGAL/license/Mesh_3.h>


namespace CGAL { namespace Mesh_3 {

template <typename Polyhedron>
struct Detect_polylines;

template <typename Polyhedron,
          typename Polyline_and_context,
          typename Polylines_output_iterator>
Polylines_output_iterator
detect_polylines(Polyhedron* pMesh, 
                 Polylines_output_iterator out_it);

} // end namespace CGAL::Mesh_3
} // end namespace CGAL


#endif // CGAL_DETECT_POLYLINES_IN_POLYHEDRA_FWD_H
