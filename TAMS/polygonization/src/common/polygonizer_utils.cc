/*
 * polygonizer_utils.cc
 *
 *  Created on: Sep 17, 2010
 *      Author: narunas
 */

#include "polygonizer_utils.hh"
#include "boost/shared_ptr.hpp"
#include "positionVector.hh"
#include "uncertainPlane.hh"
#include "segmentation_attributes.hh"

using namespace jir;
using namespace jir::polygonization;
using namespace jir::segmentation;
using namespace jir::geometry;
using namespace std;
using namespace boost;
using namespace jir::planar_patches_interface;

void
PolygonizerUtils::projectPoints (const RangeImageInterface& range_image, Segment& seg,
                                 vector<PositionVector>& projected_points, vector<unsigned int>& indices,
                                 bool ortho_projection)
{

  const vector<unsigned int>& segment_indices = seg.getSegmentIndices ();
  boost::shared_ptr<SerializableAttribute> attr;

  if (seg.getAttributes ().getAttribute (Segment::UNCERTAIN_PLANE_ATTRIBUTE, attr))
  {
    UNCERTAIN_PLANE_ATTRIBUTE_TYPE uncertain_plane_ptr =
        shared_dynamic_cast<UNCERTAIN_PLANE_ATTRIBUTE_SERIALIZABLE_TYPE> (attr)->getAttributeValue ();
    const Plane<double>& plane = (*uncertain_plane_ptr).getPlane ();

    projected_points.clear ();
    projected_points.reserve (segment_indices.size ());

    indices.clear ();
    indices.reserve (segment_indices.size ());

    for (unsigned int idx = 0; idx < segment_indices.size (); ++idx)
    {
      if (range_image.isValid (segment_indices[idx]))
      {
        projected_points.push_back (
            PositionVector (range_image.getX (segment_indices[idx]), range_image.getY (segment_indices[idx]),
                            range_image.getZ (segment_indices[idx])));
        indices.push_back (idx);
        plane.project (projected_points.back ());
      }
    }
  }
}

