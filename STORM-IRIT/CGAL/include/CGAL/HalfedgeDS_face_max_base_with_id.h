// Copyright (c) 2007  GeometryFactory (France).  All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/HalfedgeDS/include/CGAL/HalfedgeDS_face_max_base_with_id.h $
// $Id: HalfedgeDS_face_max_base_with_id.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Andreas Fabri, Fernando Cacciola

#ifndef CGAL_HALFEDGEDS_FACE_MAX_BASE_WITH_ID_H
#define CGAL_HALFEDGEDS_FACE_MAX_BASE_WITH_ID_H 1

#include <CGAL/HalfedgeDS_face_base.h>

namespace CGAL {

template < class Refs, class Pln, class ID>
class HalfedgeDS_face_max_base_with_id : public HalfedgeDS_face_base< Refs, Tag_true, Pln>
{
public:
    
    typedef HalfedgeDS_face_base< Refs, Tag_true, Pln> Base ;
    
    typedef ID size_type ;
    
private:

    size_type mID ;
    
public:

    HalfedgeDS_face_max_base_with_id() : mID ( size_type(-1) ) {}
    HalfedgeDS_face_max_base_with_id( Pln const& p) : Base(p), mID ( size_type(-1) ) {}
    HalfedgeDS_face_max_base_with_id( Pln const& p, size_type i ) : Base(p), mID (i) {}
    
    size_type&       id()       { return mID; }
    size_type const& id() const { return mID; }
};

} //namespace CGAL

#endif // CGAL_HALFEDGEDS_FACE_MAX_BASE_WITH_ID_H //
// EOF //
