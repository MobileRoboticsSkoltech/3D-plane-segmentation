// Copyright (c) 2006  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Surface_mesh_simplification/include/CGAL/Surface_mesh_simplification/edge_collapse.h $
// $Id: edge_collapse.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H 1

#include <CGAL/license/Surface_mesh_simplification.h>


#include <CGAL/boost/graph/properties.h>
#include <CGAL/boost/graph/Named_function_parameters.h>

#include <CGAL/Surface_mesh_simplification/Detail/Edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Detail/Common.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/LindstromTurk.h>

namespace CGAL {

namespace Surface_mesh_simplification
{

template<class TM
        ,class ShouldStop
        ,class VertexIndexMap
        ,class VertexPointMap
        ,class EdgeIndexMap
        ,class EdgeIsConstrainedMap
        ,class GetCost
        ,class GetPlacement
        ,class Visitor
        >
int edge_collapse ( TM&                       aSurface
                  , ShouldStop           const& aShould_stop
                  // optional mesh information policies 
                  , VertexIndexMap       const& aVertex_index_map     // defaults to get(vertex_index,aSurface)
                  , VertexPointMap       const& aVertex_point_map     // defaults to get(vertex_point,aSurface)
                  , EdgeIndexMap         const& aEdge_index_map       // defaults to get(edge_index,aSurface)
                  , EdgeIsConstrainedMap const& aEdge_is_constrained_map   // defaults to No_constrained_edge_map<TM>()
                  
                  // optional strategy policies - defaults to LindstomTurk
                  , GetCost              const& aGet_cost
                  , GetPlacement         const& aGet_placement
                  
                  , Visitor                     aVisitor
                  ) 
{
  typedef EdgeCollapse< TM
                      , ShouldStop
                      , VertexIndexMap
                      , VertexPointMap
                      , EdgeIndexMap
                      , EdgeIsConstrainedMap
                      , GetCost
                      , GetPlacement
                      , Visitor
                      >
                      Algorithm;
                      
  Algorithm algorithm( aSurface
                     , aShould_stop
                     , aVertex_index_map
                     , aVertex_point_map
                     , aEdge_index_map
                     , aEdge_is_constrained_map
                     , aGet_cost
                     , aGet_placement
                     , aVisitor
                     ) ;
                     
  return algorithm.run();
}                          


struct Dummy_visitor
{
  template<class TM>                                 void OnStarted( TM& ) const {} 
  template<class TM>                                 void OnFinished ( TM& ) const {} 
  template<class Profile>                             void OnStopConditionReached( Profile const& ) const {} 
  template<class Profile, class OFT>                  void OnCollected( Profile const&, OFT const& ) const {}                
  template<class Profile, class OFT, class Size_type> void OnSelected( Profile const&, OFT const&, Size_type, Size_type ) const {}                
  template<class Profile, class OPoint>               void OnCollapsing(Profile const&, OPoint const& ) const {}                
  template<class Profile, class VH>                   void OnCollapsed( Profile const&, VH ) const {}
  template<class Profile>                             void OnNonCollapsable(Profile const& ) const {}                
} ;

template<class TM, class ShouldStop, class P, class T, class R>
int edge_collapse ( TM& aSurface
                  , ShouldStop const& aShould_stop
                  , Named_function_parameters<P,T,R> const& aParams 
                  ) 
{
  using parameters::choose_parameter;
  using parameters::get_parameter;
  
  LindstromTurk_params lPolicyParams ;
  
  internal_np::graph_visitor_t vis = internal_np::graph_visitor_t() ;

  return edge_collapse(aSurface
                      ,aShould_stop
                      ,choose_parameter(get_parameter(aParams,internal_np::vertex_index), get_const_property_map(boost::vertex_index, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::vertex_point),get_property_map(vertex_point, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::halfedge_index),get_const_property_map(boost::halfedge_index, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::edge_is_constrained),No_constrained_edge_map<TM>())
                      ,choose_parameter(get_parameter(aParams,internal_np::get_cost_policy), LindstromTurk_cost<TM>())
                      ,choose_parameter(get_parameter(aParams,internal_np::get_placement_policy), LindstromTurk_placement<TM>())
                      ,choose_parameter(get_parameter(aParams,vis), Dummy_visitor())
                      );

}
  template<class TM, class ShouldStop, class GT, class P, class T, class R>
int edge_collapse ( TM& aSurface
                  , ShouldStop const& aShould_stop
                  , Named_function_parameters<P,T,R> const& aParams 
                  ) 
{
  using parameters::choose_parameter;
  using parameters::get_parameter;
  
  LindstromTurk_params lPolicyParams ;
  
  internal_np::graph_visitor_t vis = internal_np::graph_visitor_t() ;
    
  return edge_collapse(aSurface
                      ,aShould_stop
                      ,choose_parameter(get_parameter(aParams,internal_np::vertex_index), get_const_property_map(boost::vertex_index, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::vertex_point),get_property_map(vertex_point, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::halfedge_index),get_const_property_map(boost::halfedge_index, aSurface))
                      ,choose_parameter(get_parameter(aParams,internal_np::edge_is_constrained),No_constrained_edge_map<TM>())
                      ,choose_parameter(get_parameter(aParams,internal_np::get_cost_policy), LindstromTurk_cost<TM>())
                      ,choose_parameter(get_parameter(aParams,internal_np::get_placement_policy), LindstromTurk_placement<TM>())
                      ,choose_parameter(get_parameter(aParams,vis), Dummy_visitor())
                      );

}

template<class TM, class ShouldStop>
int edge_collapse ( TM& aSurface, ShouldStop const& aShould_stop ) 
{
  return edge_collapse(aSurface,aShould_stop, CGAL::parameters::halfedge_index_map(get(boost::halfedge_index,aSurface)));
}

  template<class TM, class ShouldStop, class GT>
  int edge_collapse ( TM& aSurface, ShouldStop const& aShould_stop) 
{
  return edge_collapse(aSurface,aShould_stop, CGAL::parameters::halfedge_index_map(get(boost::halfedge_index,aSurface)));
}

} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_EDGE_COLLAPSE_H //
// EOF //
 
