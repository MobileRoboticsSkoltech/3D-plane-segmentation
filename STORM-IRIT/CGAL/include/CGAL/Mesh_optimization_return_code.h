// Copyright (c) 2009 INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Mesher_level/include/CGAL/Mesh_optimization_return_code.h $
// $Id: Mesh_optimization_return_code.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Stephane Tayeb
//
//******************************************************************************
// File Description : 
//******************************************************************************

#ifndef CGAL_MESH_OPTIMIZATION_RETURN_CODE_H
#define CGAL_MESH_OPTIMIZATION_RETURN_CODE_H

namespace CGAL {

enum Mesh_optimization_return_code
{
  MESH_OPTIMIZATION_UNKNOWN_ERROR=-1,
  BOUND_REACHED=0,
  TIME_LIMIT_REACHED,
  CANT_IMPROVE_ANYMORE,
  CONVERGENCE_REACHED,
  MAX_ITERATION_NUMBER_REACHED,
  ALL_VERTICES_FROZEN
};


} //namespace CGAL

#endif // CGAL_MESH_OPTIMIZATION_RETURN_CODE_H
