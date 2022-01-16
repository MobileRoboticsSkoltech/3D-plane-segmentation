// Copyright (c) 2014  
// Utrecht University (The Netherlands),
// ETH Zurich (Switzerland),
// INRIA Sophia-Antipolis (France),
// Max-Planck-Institute Saarbruecken (Germany),
// and Tel-Aviv University (Israel).  All rights reserved. 
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Number_types/include/CGAL/Exact_rational.h $
// $Id: Exact_rational.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
//
// Author(s)     : Laurent Rineau

#include <CGAL/internal/Exact_type_selector.h>

namespace CGAL {

/*!
\ingroup nt_cgal

`Exact_rational` is an exact rational number type, constructible from `double`.

It is a typedef of another number type. Its exact definition depends on
the availability the third-party libraries %GMP, %CORE, and %LEDA. %CGAL must
be configured with at least one of those libraries.

\cgalModels `Field` 
\cgalModels `RealEmbeddable` 
\cgalModels `Fraction` 
\cgalModels `FromDoubleConstructible` 

*/
#if DOXYGEN_RUNNING

typedef unspecified_type Exact_rational;

#else // not DOXYGEN_RUNNING

typedef internal::Exact_field_selector<double>::Type Exact_rational;

#endif

} /* end namespace CGAL */
