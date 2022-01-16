// Copyright (c) 2006-2009 Max-Planck-Institute Saarbruecken (Germany).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Arithmetic_kernel/include/CGAL/Arithmetic_kernel/Arithmetic_kernel_base.h $
// $Id: Arithmetic_kernel_base.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Michael Hemmer <hemmer@mpi-inf.mpg.de> 
//
// ============================================================================
//
//    \brief provide base class for Arithmetic_kernel  
//



#ifndef CGAL_ARITHMETIC_KERNEL_ARITHMETIC_KERNEL_BASE_H
#define CGAL_ARITHMETIC_KERNEL_ARITHMETIC_KERNEL_BASE_H

#include <CGAL/tags.h>

namespace CGAL {
namespace internal{

class Arithmetic_kernel_base{
public:
  typedef CGAL::Null_tag Integer;
  typedef CGAL::Null_tag Rational;
  typedef CGAL::Null_tag Field_with_sqrt;
  typedef CGAL::Null_tag Field_with_kth_root;
  typedef CGAL::Null_tag Field_with_root_of;
  typedef CGAL::Null_tag Bigfloat;
  typedef CGAL::Null_tag Bigfloat_interval;
//  typedef CGAL::Null_tag Exact_float_number;
};

}// namespace internal
} //namespace CGAL

#endif // CGAL_ARITHMETIC_KERNEL_ARITHMETIC_KERNEL_BASE_H
