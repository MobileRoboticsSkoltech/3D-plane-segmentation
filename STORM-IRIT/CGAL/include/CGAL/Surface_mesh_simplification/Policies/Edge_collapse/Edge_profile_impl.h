// Copyright (c) 2006  GeometryFactory (France). All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Surface_mesh_simplification/include/CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_profile_impl.h $
// $Id: Edge_profile_impl.h c4c1a0b 2019-10-19T16:00:05+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Fernando Cacciola <fernando.cacciola@geometryfactory.com>
//
#ifndef CGAL_SURFACE_MESH_SIMPLIFICATION_DETAIL_EDGE_PROFILE_IMPL_H
#define CGAL_SURFACE_MESH_SIMPLIFICATION_DETAIL_EDGE_PROFILE_IMPL_H 1

#include <CGAL/license/Surface_mesh_simplification.h>


namespace CGAL {

namespace Surface_mesh_simplification
{

  template<class TM, class VertexPointMap>

template<class VertexIdxMap
        ,class EdgeIdxMap
        >
  Edge_profile<TM,VertexPointMap>::Edge_profile ( halfedge_descriptor  const& aV0V1
                                , TM&                    aSurface
                                , VertexIdxMap     const& 
                                , VertexPointMap const& aVertex_point_map
                                , EdgeIdxMap       const&
                                , bool has_border

                                )
  :
   mV0V1(aV0V1)
  ,mSurface(boost::addressof(aSurface))
    , mvpm(aVertex_point_map)
  
{
  CGAL_PROFILER("Edge_profile constructor calls");

  mLink.reserve(12);
  mTriangles.reserve(16);
  mV1V0 = opposite(v0_v1(),surface_mesh());
  
  mV0 = source(v0_v1(),surface_mesh());
  mV1 = target(v0_v1(),surface_mesh());
  
  CGAL_assertion( mV0 != mV1 );
  
  mP0 = get(vertex_point_map(),mV0);
  mP1 = get(vertex_point_map(),mV1);
  
  mIsBorderV0V1 = is_border(v0_v1());
  mIsBorderV1V0 = is_border(v1_v0());
  
  if ( left_face_exists() ) 
  {
    CGAL_SURF_SIMPL_TEST_assertion( ! is_border(mV0V1) ) ;

    mVLV0 = prev(v0_v1(),surface_mesh());
    mV1VL = next(v0_v1(),surface_mesh());
    mVL   = target(v1_vL(),surface_mesh());
    
    CGAL_SURF_SIMPL_TEST_assertion( mV0 != mVL );
    CGAL_SURF_SIMPL_TEST_assertion( mVL == source(vL_v0(),surface_mesh()) );
  }
  else
  {
    CGAL_SURF_SIMPL_TEST_assertion( is_border(mV0V1) ) ;
  }
  
  if ( right_face_exists() )
  {
    CGAL_SURF_SIMPL_TEST_assertion( ! is_border(mV1V0) ) ;

    mV0VR = next(v1_v0(),surface_mesh());
    mVRV1 = prev(v1_v0(),surface_mesh());
    mVR   = target(v0_vR(),surface_mesh());
    
    CGAL_SURF_SIMPL_TEST_assertion( mV0 != mVR );
    CGAL_SURF_SIMPL_TEST_assertion( mVR == source(vR_v1(),surface_mesh()) );
  }
  else
  {
    CGAL_SURF_SIMPL_TEST_assertion( is_border(mV1V0) ) ;
  }
  
  if(has_border){
    Extract_borders();
  }
}


  template<class TM, class VertexPointMap>
  void Edge_profile<TM,VertexPointMap>::Extract_borders()
{
  halfedge_descriptor e = mV0V1;
  halfedge_descriptor oe = opposite(e, surface_mesh());
  bool b;
  if((b = is_border(e)) || is_border(oe)){
    mBorderEdges.push_back(b?e:oe);
  }
  e = next(oe,surface_mesh());
  oe = opposite(e,surface_mesh());
  while(e != mV0V1){
    if((b = is_border(e)) || is_border(oe)){
      mBorderEdges.push_back(b?e:oe);
    }
    e = next(oe,surface_mesh());
    oe = opposite(e,surface_mesh());
  }
  e = opposite(next(e,surface_mesh()),surface_mesh());
  oe = opposite(e,surface_mesh());
    while(e != mV0V1){
    if((b = is_border(e)) || is_border(oe)){
      mBorderEdges.push_back(b?e:oe);
    } 
    e = opposite(next(e,surface_mesh()),surface_mesh());
    oe = opposite(e,surface_mesh());
    }
}



// Extract all triangles (its normals) and vertices (the link) around the collapsing edge p_q
//
  template<class TM, class VertexPointMap>
  void Edge_profile<TM,VertexPointMap>::Extract_triangles_and_link()
{
  #ifdef CGAL_SMS_EDGE_PROFILE_ALWAYS_NEED_UNIQUE_VERTEX_IN_LINK
  std::set<vertex_descriptor> vertex_already_inserted;
  #endif
  // look at the two faces or holes adjacent to edge (v0,v1)
  // and at the opposite vertex if it exists
  halfedge_descriptor endleft = next(v1_v0(), surface_mesh());
  halfedge_descriptor endright = next(v0_v1(), surface_mesh());

  if( left_face_exists() )
    mTriangles.push_back(Triangle(v0(),v1(),vL()) ) ;
  if( right_face_exists() )
    mTriangles.push_back(Triangle(v1(),v0(),vR()) ) ;

  // counterclockwise around v0
  halfedge_descriptor e02 = opposite(prev(v0_v1(),surface_mesh()), surface_mesh());
  vertex_descriptor v=target(e02,surface_mesh()), v2=v;
  while(e02 != endleft) {
    #ifdef CGAL_SMS_EDGE_PROFILE_ALWAYS_NEED_UNIQUE_VERTEX_IN_LINK
    if (vertex_already_inserted.insert(v2).second)
    #endif
    mLink.push_back(v2);
    bool is_b = is_border(e02);
    e02 = opposite(prev(e02,surface_mesh()), surface_mesh());
    v = target(e02,surface_mesh());
    if( !is_b )
      mTriangles.push_back(Triangle(v,v0(),v2) ) ;
    v2 = v;
  }

  e02 = opposite(prev(v1_v0(),surface_mesh()), surface_mesh());
  if(target(e02, surface_mesh())!=v ) // add the vertex if it is not added in the following loop
  {
    #ifdef CGAL_SMS_EDGE_PROFILE_ALWAYS_NEED_UNIQUE_VERTEX_IN_LINK
    if (vertex_already_inserted.insert(v).second)
    #endif
    mLink.push_back(v);
  }

  // counterclockwise around v1
  v2 = target(e02,surface_mesh());
  v = v2;
  while(e02 != endright) {
    #ifdef CGAL_SMS_EDGE_PROFILE_ALWAYS_NEED_UNIQUE_VERTEX_IN_LINK
    if (vertex_already_inserted.insert(v2).second)
    #endif
    mLink.push_back(v2);
    bool is_b = is_border(e02);
    e02 = opposite(prev(e02,surface_mesh()), surface_mesh());
    v = target(e02,surface_mesh());
    if( !is_b ){
      mTriangles.push_back(Triangle(v,v1(),v2) ) ;
    }
    v2 = v;
  }

  if(mLink.empty() || //handle link of an isolated triangle
     target(opposite(prev(v0_v1(),surface_mesh()), surface_mesh()), surface_mesh())!=v)
  {
    #ifdef CGAL_SMS_EDGE_PROFILE_ALWAYS_NEED_UNIQUE_VERTEX_IN_LINK
    if (vertex_already_inserted.insert(v).second)
    #endif
    mLink.push_back(v);
  }

  CGAL_assertion(!mLink.empty());
}

} // namespace Surface_mesh_simplification

} //namespace CGAL

#endif // CGAL_SURFACE_MESH_SIMPLIFICATION_DETAIL_EDGE_PROFILE_IMPL_H
// EOF //
 
