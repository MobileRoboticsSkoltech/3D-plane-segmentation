// Copyright (c) 2014-2017 GeometryFactory Sarl (France)
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org)
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-5.0.2/STL_Extension/include/CGAL/Hash_handles_with_or_without_timestamps.h $
// $Id: Hash_handles_with_or_without_timestamps.h 52164b1 2019-10-19T15:34:59+02:00 Sébastien Loriot
// SPDX-License-Identifier: LGPL-3.0-or-later OR LicenseRef-Commercial
//
// Author(s)     : Laurent Rineau,
//                 Mael Rouxel-Labbé

#ifndef CGAL_HASH_HANDLES_WITH_OR_WITHOUT_TIMESTAMPS_H
#define CGAL_HASH_HANDLES_WITH_OR_WITHOUT_TIMESTAMPS_H

#include <CGAL/Has_timestamp.h>

#include <boost/functional/hash.hpp>

#include <cstddef>
#include <iterator>

namespace CGAL {

struct Hash_handles_with_or_without_timestamps
{
  template <typename Handle>
  std::size_t operator()(const Handle h) const
  {
    typedef typename std::iterator_traits<Handle>::value_type Type;

    return hash(h, Boolean_tag<CGAL::internal::Has_timestamp<Type>::value>());
  }

  template<typename Handle>
  std::size_t hash(const Handle h, Tag_false) const
  {
    return boost::hash_value(&*h);
  }

  template<typename Handle>
  std::size_t hash(const Handle h, Tag_true) const
  {
    return h->time_stamp();
  }
};

} // namespace CGAL

#endif // CGAL_HASH_HANDLES_WITH_OR_WITHOUT_TIMESTAMPS_H
