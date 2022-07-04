
#ifndef _OPENDRIVE_GEO_ARC_HH
#define _OPENDRIVE_GEO_ARC_HH
#include "OdrGeoNode.hh"
namespace OpenDrive
{
  class GeoArc : public GeoNode
  {
  public:
    double mCurv;
  public:
    explicit GeoArc();
    explicit GeoArc(GeoArc *Nf2Ao);
    virtual ~GeoArc();
    virtual void
    printData() const;
    bool readIntern(ReaderXML *F3vnM);
    virtual double s2curvature(
    const double &BJBDA);
    virtual bool ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW);
    virtual bool ds2dh(const double &rganP, double &nZWMW);
    virtual Node *getCopy(bool Mupxf = false);
    virtual bool xy2st(const double &x, const double &y,
                       double &BJBDA, double &rkiXc);
    virtual bool containsPos(const double &x, const double &y);
    virtual void setHeader(GeoHeader *hdr);
    virtual double getMaxCurvatureInRange(const double &yhFkv, const double &pwK_y);
  private:
    double mCtrX;
    double mCtrY;
  private
      :
    bool init();
  };
} // namespace OpenDrive
#endif
