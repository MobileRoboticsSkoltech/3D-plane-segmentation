#include "planar_segments_matcher/correspondence_pairs.h"
#include "planar_segments_matcher/correspondences.h"

using namespace std;

namespace tams
{
  void
  CorrespondencePairs::insert (size_t keyi, size_t keyj, size_t mapi, size_t mapj, doublemapvalue)
  {
    MapType::insert (MapPairType (pair<size_t, size_t> (keyi, keyj), PairWithValue (mapi, mapj, mapvalue)));
    swapped.insert (MapPairType (pair<size_t, size_t> (mapi, mapj), PairWithValue (keyi, keyj, mapvalue)));
  }

  void
  CorrespondencePairs::intersect (MapType::const_iterator& it, Correspondences& out) const
  {
    const CorrespondencePairs::PairType& p1 = it->first;
    const CorrespondencePairs::PairType& p2 = it->second;

    set<PairType> p1Mapped;
    pair<CorrespondencePairs::const_iterator, CorrespondencePairs::const_iterator> p1MappedIt = equal_range (p1);
    for (CorrespondencePairs::const_iterator inIt = p1MappedIt.first; inIt != p1MappedIt.second; ++inIt)
    {
      p1Mapped.insert (inIt->second);
    }
    p1MappedIt = swapped.equal_range (p1);
    for (CorrespondencePairs::const_iterator inIt = p1MappedIt.first; inIt != p1MappedIt.second; ++inIt)
    {
      p1Mapped.insert (inIt->second);
    }

    set<PairType> p2Mapped;
    pair<CorrespondencePairs::const_iterator, CorrespondencePairs::const_iterator> p2MappedIt = equal_range (p2);
    for (CorrespondencePairs::const_iterator inIt = p2MappedIt.first; inIt != p2MappedIt.second; ++inIt)
    {
      p2Mapped.insert (inIt->second);
    }
    p2MappedIt = swapped.equal_range (p2);
    for (CorrespondencePairs::const_iterator inIt = p2MappedIt.first; inIt != p2MappedIt.second; ++inIt)
    {
      p2Mapped.insert (inIt->second);
    }

    /*set_intersection( p1Mapped.begin(), p1Mapped.end(),
     p2Mapped.begin(), p2Mapped.end(),
     insert_iterator< set<PairType> >(out,out.begin()));
     */

    set_intersection (p1Mapped.begin (), p1Mapped.end (), p2Mapped.begin (), p2Mapped.end (), back_inserter (out));

    out.recreateHash ();

  }

  ostream&
  CorrespondencePairs::output (ostream& os) const
  {
    for (MapType::const_iterator it = begin (); it != end (); ++it)
    {
      os << "(" << it->first.first << ", " << it->first.second << ") <-> " << "(" << it->second.first << ", "
          << it->second.second << ") " << "\t\t value= " << it->second.value << endl;
    }
    os << "Total: " << size () << " mapped pairs." << endl;
    return os;
  }

}

