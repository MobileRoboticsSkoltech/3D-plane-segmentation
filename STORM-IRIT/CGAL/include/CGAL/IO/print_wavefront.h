// Copyright (c) 1997  ETH Zurich (Switzerland).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Polyhedron_IO/include/CGAL/IO/print_wavefront.h $
// $Id: print_wavefront.h 254d60f 2019-10-19T15:23:19+02:00 Sébastien Loriot
// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-Commercial
// 
//
// Author(s)     : Lutz Kettner  <kettner@mpi-sb.mpg.de>

#ifndef CGAL_IO_PRINT_WAVEFRONT_H
#define CGAL_IO_PRINT_WAVEFRONT_H 1

#include <CGAL/license/Polyhedron.h>


#include <CGAL/IO/File_writer_wavefront.h>
#include <CGAL/IO/generic_print_polyhedron.h>
#include <CGAL/Polyhedron_3.h>
#include <iostream>

namespace CGAL {

template <class Polyhedron>
void print_polyhedron_wavefront( std::ostream& out, const Polyhedron& P) {
    File_writer_wavefront  writer;
    generic_print_polyhedron( out, P, writer);
}

// Deprecated global functions, replaced with functions above

template < class Traits,
           class Items,
           template < class T, class I, class A>
           class HDS, class Alloc>
void
print_wavefront( std::ostream& out, 
                 const Polyhedron_3<Traits,Items,HDS,Alloc>& P) {
    File_writer_wavefront  writer;
    generic_print_polyhedron( out, P, writer);
}

} //namespace CGAL
#endif // CGAL_IO_PRINT_WAVEFRONT_H //
// EOF //
