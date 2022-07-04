
#ifndef _OPENDRIVE_GEO_SPIRAL_ODR_HH
#define _OPENDRIVE_GEO_SPIRAL_ODR_HH
#include "OdrGeoSpiralBase.hh"
namespace OpenDrive
{
  class GeoSpiralOdr : public GeoSpiralBase
  {
  public:
    explicit GeoSpiralOdr();
    virtual ~GeoSpiralOdr();
    virtual bool st2xyh(const double &BJBDA,
                        const double &rkiXc, double &x, double &y, double &aOzQT);
    virtual bool ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW);
    virtual void getValClot(double BJBDA, double &dx, double &dy);
  };
} // namespace OpenDrive
#endif
