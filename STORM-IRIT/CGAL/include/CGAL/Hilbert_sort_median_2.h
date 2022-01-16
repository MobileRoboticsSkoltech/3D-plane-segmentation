// Copyright (c) 2007  INRIA Sophia-Antipolis (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/Spatial_sorting/include/CGAL/Hilbert_sort_median_2.h $
// $Id: Hilbert_sort_median_2.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Christophe Delage

#ifndef CGAL_HILBERT_SORT_MEDIAN_2_H
#define CGAL_HILBERT_SORT_MEDIAN_2_H

#include <CGAL/config.h>
#include <functional>
#include <cstddef>
#include <CGAL/Hilbert_sort_base.h>

namespace CGAL {

namespace internal {
    template <class K, int x, bool up> struct Hilbert_cmp_2;

    template <class K, int x>
    struct Hilbert_cmp_2<K,x,true>
        : public CGAL::cpp98::binary_function<typename K::Point_2,
                                              typename K::Point_2, bool>
    {
        typedef typename K::Point_2 Point;
        K k;
        Hilbert_cmp_2 (const K &_k = K()) : k(_k) {}
        bool operator() (const Point &p, const Point &q) const
        { 
            return Hilbert_cmp_2<K,x,false> (k) (q, p);
        }
    };
    
    template <class K>
    struct Hilbert_cmp_2<K,0,false>
        : public CGAL::cpp98::binary_function<typename K::Point_2,
                                              typename K::Point_2, bool>
    {
        typedef typename K::Point_2 Point;
        K k;
        Hilbert_cmp_2 (const K &_k = K()) : k(_k) {}
        bool operator() (const Point &p, const Point &q) const
        { 
	  return k.less_x_2_object() (p, q);
        }
    };
    
    template <class K>
    struct Hilbert_cmp_2<K,1,false>
        : public CGAL::cpp98::binary_function<typename K::Point_2,
                                              typename K::Point_2, bool>
    {
        typedef typename K::Point_2 Point;
        K k;
        Hilbert_cmp_2 (const K &_k = K()) : k(_k) {}
        bool operator() (const Point &p, const Point &q) const
        { 
            return k.less_y_2_object() (p, q);
        }
    };
}

template <class K>
class Hilbert_sort_median_2
{
public:
    typedef K Kernel;
    typedef typename Kernel::Point_2 Point;
    
private:
    Kernel _k;
    std::ptrdiff_t _limit;

    template <int x, bool up> struct Cmp : public internal::Hilbert_cmp_2<Kernel,x,up>
    { Cmp (const Kernel &k) : internal::Hilbert_cmp_2<Kernel,x,up> (k) {} };

public:
    Hilbert_sort_median_2 (const Kernel &k = Kernel(), std::ptrdiff_t limit = 1)
        : _k(k), _limit (limit)
    {}

    template <int x, bool upx, bool upy, class RandomAccessIterator>
    void sort (RandomAccessIterator begin, RandomAccessIterator end) const
    {
        const int y = (x + 1) % 2;
        if (end - begin <= _limit) return;

        RandomAccessIterator m0 = begin, m4 = end;

        RandomAccessIterator m2 = internal::hilbert_split (m0, m4, Cmp< x,  upx> (_k));
        RandomAccessIterator m1 = internal::hilbert_split (m0, m2, Cmp< y,  upy> (_k));
        RandomAccessIterator m3 = internal::hilbert_split (m2, m4, Cmp< y, !upy> (_k));

        sort<y, upy, upx> (m0, m1);
        sort<x, upx, upy> (m1, m2);
        sort<x, upx, upy> (m2, m3);
        sort<y,!upy,!upx> (m3, m4);
    }

    template <class RandomAccessIterator>
    void operator() (RandomAccessIterator begin, RandomAccessIterator end) const
    {
        sort <0, false, false> (begin, end);
    }
};

} // namespace CGAL

#endif//CGAL_HILBERT_SORT_MEDIAN_2_H
