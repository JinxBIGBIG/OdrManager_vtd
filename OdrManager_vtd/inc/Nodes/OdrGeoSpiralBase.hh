
#ifndef _OPENDRIVE_GEO_SPIRAL_BASE_HH
#define _OPENDRIVE_GEO_SPIRAL_BASE_HH
#include "OdrGeoNode.hh"
namespace OpenDrive
{
  class GeoSpiralBase : public GeoNode
  {
  public:
    double mStartCurv;
    double mEndCurv;

  public:
    explicit GeoSpiralBase();
    virtual ~GeoSpiralBase();
    explicit GeoSpiralBase(GeoSpiralBase *Nf2Ao);
    virtual void printData() const;
    bool readIntern(ReaderXML *F3vnM);
    virtual bool xy2st(const double &x, const double &y,
                       double &BJBDA, double &rkiXc);
    virtual bool st2xyh(const double &BJBDA, const double &rkiXc, double &x, double &y, double &aOzQT);
    virtual bool ds2dxdydh(const double &rganP,
                           double &dx, double &dy, double &nZWMW);
    virtual bool ds2dh(const double &rganP, double &
                                                nZWMW);
    virtual double s2curvature(const double &BJBDA);
    virtual void getValClot(
    double BJBDA, double &dx, double &dy);
    virtual Node *getCopy(bool Mupxf = false);
    virtual void setHeader(GeoHeader *hdr);
    virtual double getMaxCurvatureInRange(const double &yhFkv, const double &pwK_y);
  private:
    double mS0;
    double mClStartX;
    double
    mClStartY;
    bool mType2;
    bool mSplitCalc;
    double mCtrX;
    double mCtrY;
    double mCtrH;

  protected:
    double mCurvDot;
    bool mValid;
  protected:
    bool init();
  };
} // namespace OpenDrive
#endif
