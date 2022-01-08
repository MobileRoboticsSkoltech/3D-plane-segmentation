#include "planar_segments_matcher/correspondences.h"

using namespace tams;

void
Correspondences::recreateHash (void)
{
  _uniqueLs.clear ();
  _uniqueRs.clear ();
  Type::const_iterator it = begin ();
  for (; it != end (); ++it)
  {
    _uniqueLs.insert (it->first);
    _uniqueRs.insert (it->second);
  }
}
