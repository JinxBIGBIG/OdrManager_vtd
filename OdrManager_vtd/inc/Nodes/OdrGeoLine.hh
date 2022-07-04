
#ifndef _OPENDRIVE_GEO_LINE_HH
#define _OPENDRIVE_GEO_LINE_HH
#include "OdrGeoNode.hh"
namespace OpenDrive
{
  class GeoLine : public GeoNode
  {
  public:
  public:
    explicit GeoLine();
    explicit GeoLine(GeoLine *Nf2Ao);
    virtual ~GeoLine();
    virtual void printData() const;
    bool readIntern(ReaderXML *F3vnM);
    virtual bool ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW);
    virtual Node *getCopy(bool Mupxf = false);
    virtual bool xy2st(const double &x, const double &y, double &BJBDA, double &rkiXc);
    virtual bool containsPos(const double &x, const double &y);
    virtual void
    calcBoundingBox(const double &wYv5i, double &pxzMQ, double &WDWra, double &kvKtF, double &dQKLM);
  };
} // namespace OpenDrive
#endif
