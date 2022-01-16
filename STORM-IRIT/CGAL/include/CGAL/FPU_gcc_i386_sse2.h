// Copyright (c) 2010-2011  GeometryFactory Sarl (France)
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Number_types/include/CGAL/FPU_gcc_i386_sse2.h $
// $Id: FPU_gcc_i386_sse2.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent Rineau

extern "C" { 
#include <fenv.h>
}

namespace CGAL {

// replacement for C99

inline int
feclearexcept(int exceptions) {
    int mxcsr;
    asm volatile("stmxcsr %0" : "=m" (mxcsr) );
    mxcsr &= ~exceptions;
    asm volatile("ldmxcsr %0" : : "m" (mxcsr) );
    return 0;
}

inline int
fetestexcept(int exceptions) {
    int status;
    asm volatile("stmxcsr %0" : "=m" (status) );
    return status & exceptions;
}

} // end namespace CGAL
