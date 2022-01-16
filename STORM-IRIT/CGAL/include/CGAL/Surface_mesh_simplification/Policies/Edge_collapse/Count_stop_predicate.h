// Copyright (c) 2006  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Surface_mesh_simplification/include/CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h $
// $Id: Count_stop_predicate.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_COUNT_STOP_PREDICATE_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_COUNT_STOP_PREDICATE_H 1

#include <CGAL/license/Surface_mesh_simplification.h>


#include <CGAL/Surface_mesh_simplification/Detail/Common.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_profile.h>

namespace CGAL {

namespace Surface_mesh_simplification
{

//*******************************************************************************************************************
//                                -= stopping condition predicate =-
//
// Determines whether the simplification has finished.
// The arguments are (current_cost,vertex,vertex,is_edge,initial_pair_count,current_pair_count,surface) and the result is bool
//
//*******************************************************************************************************************

// 
// Stops when the number of edges left falls below a given number.
//
template<class TM_>    
class Count_stop_predicate
{
public:

  typedef TM_ TM ;

  //  typedef Edge_profile<TM> Profile ;
  
  typedef typename boost::graph_traits<TM>::edges_size_type size_type ;
  
  // typedef typename Kernel::FT FT ;

public :
  
  Count_stop_predicate( std::size_t aThres ) : mThres(aThres) {}
  
  template <typename F, typename Profile> 
  bool operator()( F const&         // aCurrentCost
                 , Profile const& // aEdgeProfile
                 , std::size_t    // aInitialCount
                 , std::size_t       aCurrentCount
                 ) const 
  {
    return aCurrentCount < mThres ;
  }
  
private:
  
  std::size_t mThres ;
};    

} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_POLICIES_EDGE_COLLAPSE_COUNT_STOP_PREDICATE_H //
// EOF //
 
