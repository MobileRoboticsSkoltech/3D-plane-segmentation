// Copyright (c) 2017  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Surface_mesh_simplification/include/CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h $
// $Id: Bounded_normal_change_placement.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Andreas Fabri
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_BOUNDED_NORMAL_CHANGE_PLACEMENT_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_BOUNDED_NORMAL_CHANGE_PLACEMENT_H

#include <CGAL/license/Surface_mesh_simplification.h>
#include <boost/optional.hpp>

namespace CGAL {

namespace Surface_mesh_simplification  
{

template<class Placement>
class Bounded_normal_change_placement
{
public:
    
  typedef typename Placement::TM TM ;
  
public:
  
  Bounded_normal_change_placement(const Placement& placement = Placement() )
    : mPlacement(placement)
  {}
     
  template <typename Profile> 
  boost::optional<typename Profile::Point>
  operator()( Profile const& aProfile) const
  {
    boost::optional<typename Profile::Point> op = mPlacement(aProfile);
    if(op){
      // triangles returns the triangles of the star of the vertices of the edge to collapse
      // First the two trianges incident to the edge, then the other triangles
      // The second vertex of each triangle is the vertex that gets placed
       const typename Profile::Triangle_vector& triangles = aProfile.triangles();
       if(triangles.size()>2){
         typedef typename Profile::Point Point;
         typedef typename Profile::Kernel Traits;
         typedef typename Traits::Vector_3 Vector;
         typename Profile::VertexPointMap ppmap = aProfile.vertex_point_map();
         typename Profile::Triangle_vector::const_iterator it = triangles.begin();
         if(aProfile.left_face_exists()){
           ++it; 
         }
         if(aProfile.right_face_exists()){
           ++it;
         }
         while(it!= triangles.end()){
           const typename Profile::Triangle& t = *it;
           Point p = get(ppmap,t.v0);
           Point q = get(ppmap,t.v1);
           Point r = get(ppmap,t.v2);
           Point q2 = *op;
           
           Vector eqp = Traits().construct_vector_3_object()(q,p) ;
           Vector eqr = Traits().construct_vector_3_object()(q,r) ;
           Vector eq2p = Traits().construct_vector_3_object()(q2,p) ;
           Vector eq2r = Traits().construct_vector_3_object()(q2,r) ;
           
           Vector n1 = Traits().construct_cross_product_vector_3_object()(eqp,eqr);
           Vector n2 = Traits().construct_cross_product_vector_3_object()(eq2p,eq2r);
           if(! is_positive(Traits().compute_scalar_product_3_object()(n1, n2))){
             return boost::optional<typename Profile::Point>();
           }
           ++it;
         }
       }
    }
    return op;
  }
  
private:

  Placement  mPlacement ;

};


} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_BOUNDED_NORMAL_CHANGE_PLACEMENT_H

 
