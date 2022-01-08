#ifndef __CONVEX_OUTLINER_HH
#define __CONVEX_OUTLINER_HH

#include "polygonizer_interface.hh"
#include "range_image_interface.hh"
#include "segmentation.hh"

namespace jir
{
  namespace polygonization
  {

    class ConvexPolygonizer : public PolygonizerInterface
    {

    public:
      virtual void
      polygonize (segmentation::Segmentation& segmentation);

    protected:
      virtual void
      _polygonize (const planar_patches_interface::RangeImageInterface& range_image, segmentation::Segment& seg);

    };

  }
}

#endif
