#ifndef CORRESPONDENCE_PAIRS_H_
#define CORRESPONDENCE_PAIRS_H_

#include <utility>
#include <iostream>
#include <map>
#include <set>
#include <utility>
#include <algorithm>

#include "correspondences.h"

namespace tams
{
  /** \brief Modified from jir_3d_mapping which is a library of Jacobs Robotics.
    * For detail, see http://robotics.jacobs-university.de/projects/3Dmap. */
  struct PairWithValue : public std::pair<size_t, size_t>
  {
      PairWithValue (size_t i, size_t j, double v) : std::pair<size_t, size_t> (i, j), value (v)
      {
      }

      PairWithValue (const PairWithValue& cp) : std::pair<size_t, size_t> (cp), value (cp.value)
      {
      }

      PairWithValue&
      operator = (const PairWithValue& cp)
      {
        first = cp.first;
        second = cp.second;
        value = cp.value;
        return (*this);
      }

      /** This is some pair to pair metric.*/
      double value;
  };

  /** This will be used to store pairs which are consistent w.r.t each other.
   */
  class CorrespondencePairs : public std::multimap<std::pair<size_t, size_t>, PairWithValue>
  {
    public:
      typedef std::pair<size_t, size_t> PairType;
      typedef std::pair<PairType, PairWithValue> MapPairType;
      typedef std::multimap<PairType, PairWithValue> MapType;

      /**
     * Inserts (keyi, keyj) <-> ((mapi, mapj), mapvalue)
     */
      void
      insert (size_t keyi, size_t keyj, size_t mapi, size_t mapj, double mapvalue);

      /** takes the key k and the mapped pair m of itertor it and finds the intersection of
     * all elements which are mapped to both k and m.
     * The result is in out, which is not cleared but appended to.
     */
      void
      intersect (MapType::const_iterator& it, Correspondences& out) const;

      std::ostream&
      output (std::ostream&) const;

    protected:
      MapType swapped;

  };

}


#endif // CORRESPONDENCE_PAIRS_H
