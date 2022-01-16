// Copyright (c) 2007  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Spatial_sorting/include/CGAL/Hilbert_sort_base.h $
// $Id: Hilbert_sort_base.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Christophe Delage

#ifndef CGAL_HILBERT_SORT_BASE_H
#define CGAL_HILBERT_SORT_BASE_H

#include <CGAL/config.h>
#include <algorithm>
#include <CGAL/algorithm.h>

namespace CGAL {

namespace internal {

    template <class RandomAccessIterator, class Cmp>
    RandomAccessIterator
    hilbert_split (RandomAccessIterator begin, RandomAccessIterator end,
                   Cmp cmp = Cmp ())
    {
        if (begin >= end) return begin;
      #if defined(CGAL_HILBERT_SORT_WITH_MEDIAN_POLICY_CROSS_PLATFORM_BEHAVIOR)
        RandomAccessIterator middle = begin + (end - begin) / 2;
        CGAL::nth_element (begin, middle, end, cmp);
        return middle;      
      #else
        RandomAccessIterator middle = begin + (end - begin) / 2;
        std::nth_element (begin, middle, end, cmp);
        return middle;
      #endif	
      
    }
}

} // namespace CGAL

#endif//CGAL_HILBERT_SORT_BASE_H
