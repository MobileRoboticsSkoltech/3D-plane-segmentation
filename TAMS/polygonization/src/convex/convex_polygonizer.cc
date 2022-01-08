#include "convex_polygonizer.hh"
#include "polygonizer_utils.hh"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/ch_graham_andrew.h>
#include <iostream>
#include <vector>

#include "polygon3d.hh"
#include "uncertainPlane.hh"
#include "boost/shared_ptr.hpp"
#include "segmentation_attributes.hh"

using namespace std;
using namespace jir;
using namespace jir::polygonization;
using namespace jir::segmentation;
using namespace jir::geometry;
using namespace jir::planar_patches_interface;
using namespace boost;

typedef PolygonKernel::Point_2 Point;

void
ConvexPolygonizer::polygonize (segmentation::Segmentation& segmentation)
{
  vector<Segment>& segments = segmentation.getSegments ();
  const RangeImageInterface& range_image = *(segmentation.getRangeImage ());

  for (vector<Segment>::iterator it = segments.begin (); it != segments.end (); ++it)
  {
    _polygonize (range_image, *it);
  }
}

void
ConvexPolygonizer::_polygonize (const planar_patches_interface::RangeImageInterface& range_image,
                                segmentation::Segment& seg)
{

  boost::shared_ptr<SerializableAttribute> attr;

  if (seg.getAttributes ().getAttribute (Segment::UNCERTAIN_PLANE_ATTRIBUTE, attr))
  {
    UNCERTAIN_PLANE_ATTRIBUTE_TYPE uncertain_plane_ptr =
        shared_dynamic_cast<UNCERTAIN_PLANE_ATTRIBUTE_SERIALIZABLE_TYPE> (attr)->getAttributeValue ();
    const Plane<double>& plane = (*uncertain_plane_ptr).getPlane ();

    vector<PositionVector> projected_points;
    vector<unsigned int> indices;
    PolygonizerUtils::projectPoints (range_image, seg, projected_points, indices);

    if (projected_points.size () == 0)
    {
      ERR ("None of the points were projected. Skipping segment %d", seg.getSegmentId ());
      return;
    }

    PositionVector mass_center (0, 0, 0);
    for (unsigned int idx = 0; idx != projected_points.size (); ++idx)
      mass_center += projected_points[idx];
    mass_center /= (double)projected_points.size ();

    unsigned int i = 0;
    Transform tr_spatial_planar;
    for (i = 0; i < projected_points.size (); ++i)
    {
      if ((mass_center - projected_points[i]).norm () > 1e-9)
      {
        tr_spatial_planar = plane.getSpatialToPlanarTransform (mass_center, projected_points[i]);
        break;
      }
    }

    if (i == projected_points.size ())
    {
      //should not be here
      ERR ("All points of the segment are the same. Skipping segment %d", seg.getSegmentId ());
      return;
    }
    //TODO: optimize, to avoid unnecessary copying
    vector<PositionVector> planar_points;
    tr_spatial_planar (projected_points, planar_points);
    vector<Point> cgal_points (planar_points.size ());
    for (unsigned int i = 0; i < cgal_points.size (); ++i)
      cgal_points[i] = Point (planar_points[i].x (), planar_points[i].y ());

    CGALPolygon outer;
    CGAL::ch_graham_andrew (cgal_points.begin (), cgal_points.end (), std::back_inserter (outer));

    boost::shared_ptr<Polygon3D> polygon_ptr (new Polygon3D (outer, tr_spatial_planar.inverse ()));

    boost::shared_ptr<SerializableAttribute> attr (new POLYGON_ATTRIBUTE_SERIALIZABLE_TYPE (polygon_ptr));
    seg.getAttributes ().setAttribute (Segment::POLYGON_ATTRIBUTE, attr);

  }
  else
  {
    ERR ("Segment %d does not have uncertain plane attribute. It can't be polygonized!", seg.getSegmentId ());
    return;
  }
}
