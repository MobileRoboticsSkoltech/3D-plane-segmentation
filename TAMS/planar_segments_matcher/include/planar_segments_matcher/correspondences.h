#ifndef CORRESPONDENCES_H_
#define CORRESPONDENCES_H_

#include <iostream>
#include <vector>
#include <utility>
#include <set>
#include <map>

namespace tams
{
  /** \brief Modified from jir_3d_mapping which is a library of Jacobs Robotics.
    * For detail, see http://robotics.jacobs-university.de/projects/3Dmap. */
  class Correspondences : public std::vector<std::pair<unsigned int, unsigned int> >
  {
  public:
    typedef std::pair<unsigned int, unsigned int> Pair;
    typedef std::vector<std::pair<unsigned int, unsigned int> > Type;
    typedef std::multimap<unsigned int, unsigned int> Map;

    Correspondences () : Type (), uniqueLs_ (), uniqueRs_ ()
    {
    }

    Correspondences (const Correspondences& c) : Type (c), uniqueLs_ (c.uniqueLs_), uniqueRs_ (c.uniqueRs_)
    {
    }

    Correspondences&
    operator= (const Correspondences& c)
    {
      Type::operator= (c);
      uniqueLs_ = c.uniqueLs_;
      uniqueRs_ = c.uniqueRs_;
      return *this;
    }

    void
    clear (void)
    {
      Type::clear ();
      uniqueLs_.clear ();
      uniqueRs_.clear ();
    }

    /** order (left,right).
     */
    void
    insert (unsigned int l, unsigned int r)
    {
      std::pair<unsigned int, unsigned int> lr (l, r);
      push_back (lr);
      uniqueLs_.insert (l);
      uniqueRs_.insert (r);
    }

    void
    insert (const Correspondences& a)
    {
      Type::const_iterator it = a.begin ();
      for (; it != a.end (); ++it)
      {
        insert (it->first, it->second);
      }
    }

    bool
    existsAsLeft (unsigned int l) const
    {
      return (uniqueLs_.count (l) != 0);
    }

    const std::set<unsigned int>&
    uniqueLs (void) const
    {
      return uniqueLs_;
    }

    bool
    existsAsRight (unsigned int r) const
    {
      return (uniqueRs_.count (r) != 0);
    }

    const std::set<unsigned int>&
    uniqueRs (void) const
    {
      return uniqueRs_;
    }

    void
    getMapFromLeftToRight (Map& m) const
    {
      m.clear ();
      for (Type::const_iterator it = begin (); it != end (); ++it)
      {
        m.insert (std::make_pair (it->first, it->second));
      }
    }

    void
    getMapFromRightToLeft (Map& m) const
    {
      m.clear ();
      for (Type::const_iterator it = begin (); it != end (); ++it)
      {
        m.insert (std::make_pair (it->second, it->first));
      }
    }

    std::ostream&
    debug (std::ostream& os) const
    {
      Type::const_iterator it = begin ();
      for (; it != end (); ++it)
      {
        os << "\t" << (it->first) << " <-> " << (it->second) << std::endl;
      }
      return os;
    }

    void
    recreateHash (void);

  protected:
    std::set<unsigned int> uniqueLs_;
    std::set<unsigned int> uniqueRs_;
  };

}
#endif
