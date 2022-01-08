/*
 * polygonizer_utils.hh
 *
 *  Created on: Sep 17, 2010
 *      Author: narunas
 */

#ifndef POLYGONIZER_UTILS_HH_
#define POLYGONIZER_UTILS_HH_

#include "positionVector.hh"
#include "segmentation.hh"
#include "range_image_interface.hh"

namespace tams
{
  namespace polygonization
  {

    class PolygonizerUtils
    {
    public:
      static void
      projectPoints (const planar_patches_interface::RangeImageInterface& range_image, segmentation::Segment& segment,
                     std::vector<geometry::PositionVector>& projected_points, std::vector<unsigned int>& indices,
                     bool ortho_projection = true);
    };

  }
}

#endif /* POLYGONIZER_UTILS_HH_ */
