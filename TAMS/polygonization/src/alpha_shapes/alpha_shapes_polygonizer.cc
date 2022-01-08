#include "alpha_shapes_polygonizer.hh"
#include "polygon2d.hh"
#include "polygon3d.hh"
#include "uncertainPlane.hh"
#include "range_image_interface.hh"
#include "polygonizer_utils.hh"
#include <vector>
#include "segmentation_attributes.hh"

//CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Triangulation_face_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/IO/Geomview_stream.h>

//boost
#include <boost/shared_ptr.hpp>

using namespace std;
using namespace jir;
using namespace jir::segmentation;
using namespace jir::planar_patches_interface;
using namespace jir::polygonization;
using namespace jir::geometry;
using namespace boost;

//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef geometry::PolygonKernel K;

typedef CGAL::Alpha_shape_vertex_base_2<K> Av;

typedef CGAL::Triangulation_face_base_2<K> Tf;
typedef CGAL::Alpha_shape_face_base_2<K, Tf> Af;

typedef CGAL::Triangulation_default_data_structure_2<K, Av, Af> Tds;
typedef CGAL::Delaunay_triangulation_2<K, Tds> Dt;
typedef CGAL::Alpha_shape_2<Dt> Alpha_shape_2;
typedef Alpha_shape_2::Point Point;

/*
 *      Alpha Shapes 
 * 
 */

void
AlphaShapesPolygonizer::polygonize (segmentation::Segmentation& segmentation)
{
  vector<Segment>& segments = segmentation.getSegments ();
  const RangeImageInterface& range_image = *(segmentation.getRangeImage ());
  for (vector<Segment>::iterator it = segments.begin (); it != segments.end (); ++it)
  {
    _polygonize (range_image, *it);
  }
}

void
AlphaShapesPolygonizer::_polygonize (const RangeImageInterface& range_image, segmentation::Segment& seg)
{
  //const vector <unsigned int>& segment_indices = seg.getSegmentIndices();
  shared_ptr<SerializableAttribute> attr;
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

    /** compute the mass_center of this plane.*/
    PositionVector mass_center (0, 0, 0);
    for (unsigned int idx = 0; idx != projected_points.size (); ++idx)
      mass_center += projected_points[idx];
    mass_center /= (double)projected_points.size ();

    unsigned int i = 0;
    Transform tr_spatial_planar;
    for (i = 0; i < projected_points.size (); ++i)
    {
      if ((mass_center - projected_points[i]).norm () > 1e-9)
      /** why compute the distance between the mass center and each point?*/
      /** compute the ralertive coordinate of the planar patch.*/
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

    /** construct cgal 2-D points from the local coordinate frame.*/
    vector<Point> cgal_points (planar_points.size ());
    for (unsigned int i = 0; i < cgal_points.size (); ++i)
      cgal_points[i] = Point (planar_points[i].x (), planar_points[i].y ());

    /* create AlphaShape object */
    Alpha_shape_2 as (cgal_points.begin (), cgal_points.end (), Alpha_shape_2::REGULARIZED); //INTERIOR);//REGULAR);
    /* find an optimal alpha value to have only one component */
    Alpha_shape_2::Alpha_iterator opt = as.find_optimal_alpha (1);
    as.set_alpha (*opt);

    /** 没懂这段什么意思！！ */
    /* construct a graph */
    vector<Point> vertices;
    std::map<Point, vector<Point> > map;
    std::map<pair<Point, Point>, int> met;
    Alpha_shape_2::Alpha_shape_edges_iterator ase = as.alpha_shape_edges_begin ();
    while (ase != as.alpha_shape_edges_end ())
    {
      Alpha_shape_2::Edge e = *ase;
      Alpha_shape_2::Face_handle face_handle = e.first;
      int index = e.second;
      Point c1 = (Point)face_handle->vertex (as.cw (index))->point ();
      Point c2 = (Point)face_handle->vertex (as.ccw (index))->point ();
      met[pair<Point, Point> (c1, c2)]++;
      met[pair<Point, Point> (c2, c1)]++;
      map[c1].push_back (c2);
      map[c2].push_back (c1);
      ase++;
    }

    CGALPolygon outer;

    std::map<pair<Point, Point>, int>::iterator it = met.begin ();
    vector<stack<Point> > outlines;
    /* construct the outline */
    while (it != met.end ())
    {
      if (it->second > 0)
      {
        outlines.push_back (stack<Point> ());
        stack<Point> &stack = outlines[outlines.size () - 1];

        met[it->first]--;
        met[pair<Point, Point> (it->first.second, it->first.first)]--;
        Point firstPoint = it->first.first;
        stack.push (it->first.first);
        stack.push (it->first.second);

        Point cur = it->first.second;
        while (1)
        {
          int i = 0;
          Point p (0, 0);
          while (i < (int)map[cur].size ())
          {
            p = map[cur][i];
            if (met[pair<Point, Point> (cur, p)])
              break;
            i++;
          }
          if (i >= (int)map[cur].size ())
          {
            if (firstPoint != cur)
            {
              if (!stack.empty ())
              {
                stack.pop ();
                cur = stack.top ();
                continue;
              }
              stack.push (firstPoint);
            }
            break;
          }
          met[pair<Point, Point> (cur, p)]--;
          met[pair<Point, Point> (p, cur)]--;
          cur = p;
          stack.push (cur);
        }
      }
      it++;
    }

    int max = 0, nmax = 0;
    for (int i = 0; i < (int)outlines.size (); i++)
    {
      if ((int)outlines[i].size () > max)
      {
        max = outlines[i].size ();
        nmax = i;
      }
    }

    stack<Point> &oStack = outlines[nmax];
    //    while (!oStack.empty()) {
    while (oStack.size () > 1)
    {
      outer.push_back (oStack.top ());
      oStack.pop ();
    }

    /* collect all holes */
    vector<CGALPolygon> holes;
    for (int i = 0; i < (int)outlines.size (); i++)
    {
      if (i != nmax)
      {
        if (outlines[i].size () > 3)
        {
          holes.push_back (CGALPolygon ());
          CGALPolygon &oip = holes[holes.size () - 1];
          stack<Point> &points = outlines[i];
          // first point == last point, so:
          while (points.size () > 1)
          {
            oip.push_back (points.top ());
            points.pop ();
          }
        }
      }
    }
    CGALPolygonWithHoles cgal_poly (outer, holes.begin (), holes.end ()); //TODO: add constructor to Polygon3D
    boost::shared_ptr<Polygon3D> polygon_ptr (new Polygon3D (cgal_poly, tr_spatial_planar.inverse ()));
    if (_simplify)
      polygon_ptr->getPolygon2D ().makeSimple ();

    boost::shared_ptr<SerializableAttribute> attr (
        new BoostSerializableAttribute<POLYGON_ATTRIBUTE_TYPE> (polygon_ptr));
    seg.getAttributes ().setAttribute (Segment::POLYGON_ATTRIBUTE, attr);
  }
  else
  {
    ERR ("Segment %d does not have uncertain plane attribute. It can't be polygonized!", seg.getSegmentId ());
    return;
  }
}

