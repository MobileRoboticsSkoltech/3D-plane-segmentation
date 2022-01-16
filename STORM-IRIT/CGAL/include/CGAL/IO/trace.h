// Copyright (c) 2007-09  INRIA Sophia-Antipolis (France).
// All rights reserved.
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Stream_support/include/CGAL/IO/trace.h $
// $Id: trace.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s) : Laurent Saboret

#ifndef CGAL_IO_TRACE_H
#define CGAL_IO_TRACE_H

#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <fstream>

/// \cond SKIP_IN_MANUAL

// Trace utilities
// ---------------

// print_stderr() = printf-like function to print to stderr
inline void CGAL_print_stderr(const char *fmt, ...)
{
  va_list argp;
  va_start(argp, fmt);
  vfprintf(stderr, fmt, argp);
  va_end(argp);
}

// CGAL_TRACE() = printf-like function to print to stderr
// if DEBUG_TRACE is defined (ignored otherwise)
#ifdef DEBUG_TRACE
#define CGAL_TRACE  CGAL_print_stderr
#else
  #define CGAL_TRACE  if (false) CGAL_print_stderr
#endif

// CGAL_TRACE_STREAM = C++ stream that prints to std::cerr 
// if DEBUG_TRACE is defined (ignored otherwise)
#ifdef DEBUG_TRACE
  #define CGAL_TRACE_STREAM  std::cerr
#else
  #define CGAL_TRACE_STREAM  if (false) std::cerr
#endif

/// \endcond

#endif // CGAL_IO_TRACE_H
