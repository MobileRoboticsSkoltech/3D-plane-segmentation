#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/jet_smooth_point_set.h>

#include <vector>

// types
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


int main(void)
{
  // generate point set
  std::vector<Point> points;
  points.push_back(Point( 0.0, 0.0, 0.001));
  points.push_back(Point(-0.1,-0.1, 0.002));
  points.push_back(Point(-0.1,-0.2, 0.001));
  points.push_back(Point(-0.1, 0.1, 0.002));
  points.push_back(Point( 0.1,-0.1, 0.000));
  points.push_back(Point( 0.1, 0.2, 0.001));
  points.push_back(Point( 0.2, 0.0, 0.002));
  points.push_back(Point( 0.2, 0.1, 0.000));
  points.push_back(Point( 0.0,-0.1, 0.001));

  // Smoothing.
  const unsigned int nb_neighbors = 8; // default is 24 for real-life point sets
  CGAL::jet_smooth_point_set<Concurrency_tag>(points, nb_neighbors);

  return EXIT_SUCCESS;
}

