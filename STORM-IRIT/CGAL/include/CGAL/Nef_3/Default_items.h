// Copyright (c) 1997-2002  Max-Planck-Institute Saarbruecken (Germany).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Nef_3/include/CGAL/Nef_3/Default_items.h $
// $Id: Default_items.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Andreas Fabri <andreas.fabri@geometryfactory.com>

#ifndef CGAL_NEF_DEFAULT_ITEMS_H
#define CGAL_NEF_DEFAULT_ITEMS_H

#include <CGAL/license/Nef_3.h>


#include <CGAL/Nef_3/SNC_items.h>
#include <CGAL/Nef_3/SNC_indexed_items.h>
#include <CGAL/Extended_homogeneous.h>
#include <CGAL/Extended_cartesian.h>

namespace CGAL {

template<class Kernel>
struct Default_items {
  typedef CGAL::SNC_indexed_items Items;
};

template<typename NT>
struct Default_items<CGAL::Extended_homogeneous<NT> > {
  typedef CGAL::SNC_items Items;
};

template<typename NT>
struct Default_items<CGAL::Extended_cartesian<NT> > {
  typedef CGAL::SNC_items Items;
};

} //namespace CGAL
#endif
