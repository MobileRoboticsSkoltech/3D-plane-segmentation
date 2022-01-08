/*
 * polygonizer_attributes.hh
 *
 *  Created on: Jun 22, 2011
 *      Author: narunas
 */

#ifndef POLYGONIZER_ATTRIBUTES_HH_
#define POLYGONIZER_ATTRIBUTES_HH_

#include <boost/shared_ptr.hpp>
#include "polygon3d.hh"
#include "serializable_attribute.hh"

#ifdef WITH_BOOST_SERIALIZATION
#include <boost/serialization/export.hpp>
#endif

  namespace tams
  {

    typedef boost::shared_ptr<jir::geometry::Polygon3D> POLYGON_ATTRIBUTE_TYPE;
    typedef jir::planar_patches_interface::BoostSerializableAttribute<jir::polygonization::POLYGON_ATTRIBUTE_TYPE> POLYGON_ATTRIBUTE_SERIALIZABLE_TYPE;

  }

#ifdef WITH_BOOST_SERIALIZATION
BOOST_CLASS_EXPORT(jir::polygonization::POLYGON_ATTRIBUTE_SERIALIZABLE_TYPE)
#endif

#endif /* POLYGONIZER_ATTRIBUTES_HH_ */
