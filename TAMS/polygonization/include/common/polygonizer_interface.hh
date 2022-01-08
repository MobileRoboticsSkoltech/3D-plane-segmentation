#ifndef POLYGONIZER_INTERFACE_HH_
#define POLYGONIZER_INTERFACE_HH_

#include "attributes.hh"
#include "segmentation.hh"
#include "polygonizer_attributes.hh"

namespace tams
{

  class PolygonizerInterface
  {
    public:
      PolygonizerInterface ()
      {
      }
      ;
      PolygonizerInterface (const planar_patches_interface::Attributes<int>& parameters) :
        _param (parameters)
      {
      }
      ;

      virtual
      ~PolygonizerInterface ()
      {
      }
      ;

      virtual void
      polygonize (segmentation::Segmentation& segmentation) = 0;

      planar_patches_interface::Attributes<int>&
      getParameters ()
      {
        return _param;
      }
    protected:
      planar_patches_interface::Attributes<int> _param;
  };

}


#endif /* POLYGONIZER_INTERFACE_HH_ */
