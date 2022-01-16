// Copyright (c) 2006  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Surface_mesh_simplification/include/CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk_params.h $
// $Id: LindstromTurk_params.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_LINDSTROMTURK_PARAMS_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_LINDSTROMTURK_PARAMS_H

#include <CGAL/license/Surface_mesh_simplification.h>


namespace CGAL {

namespace Surface_mesh_simplification 
{

struct LindstromTurk_params
{
  LindstromTurk_params()
    :
    VolumeWeight  (0.5)
   ,BoundaryWeight(0.5)
   ,ShapeWeight   (0)
  {}
  
  LindstromTurk_params( double aVolumeWeight, double aBoundaryWeight, double aShapeWeight )
    :
    VolumeWeight  (aVolumeWeight)
   ,BoundaryWeight(aBoundaryWeight)
   ,ShapeWeight   (aShapeWeight)
  {}
    
  double VolumeWeight ;
  double BoundaryWeight ;
  double ShapeWeight ;
};
  
} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_LINDSTROMTURK_PARAMS_H
// EOF //
 
