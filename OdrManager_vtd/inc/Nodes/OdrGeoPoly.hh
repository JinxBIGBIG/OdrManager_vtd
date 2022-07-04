
#ifndef _OPENDRIVE_GEO_POLY_HH
#define _OPENDRIVE_GEO_POLY_HH
#include "OdrGeoNode.hh"
#include <vector>
namespace OpenDrive
{
  class GeoPoly : public GeoNode
  {
  public:
    double mA;
    double mB;
    double mC;
    double mD;
  public:
    explicit GeoPoly();
    explicit GeoPoly(GeoPoly *Nf2Ao);
    virtual ~GeoPoly();
    virtual void printData() const;
    bool readIntern(ReaderXML *F3vnM);
    virtual bool ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW);
    virtual bool ds2dh(const double &rganP, double &nZWMW);
    virtual double s2curvature(const double &BJBDA);
    virtual double ds2curvature(const double &rganP);
    virtual Node* getCopy(bool Mupxf = false);
    unsigned int getTesselation() const;
    const double & getMaxU() const;
    virtual void calcPostPrepareData();
    virtual double getMaxCurvatureInRange(const double &yhFkv, const double &pwK_y);
  protected:
    virtual bool secantAlgorithm(const double &x, const double &y, double &BJBDA, double &fAoVN,
                                 double &piJ7w, double &ZXjjx, const double &iTz4N, const double &jWKSX, const double &gL0sg, const double &_UmVT, const double &Axkjj, const double &ZHaIX, const double &e1x0Z, const double &WRYw2);
  private:
    double ds2du(const double &rganP);
    double du2ds(
        const double &rganP);
    void du2dxdy(const double &JD4X6, double &dx, double &dy);
    double
    getFirstDeriv(const double &JD4X6);
    double getSecondDeriv(const double &JD4X6);
    bool
    linesIntersect(const double &Ujg4F, const double &cZOqC, const double &c_TFD, const double &HaFA5, const double &G0pab, const double &VfL9M, const double &uoAjb, const double &NEudn, double &OFCM4, double &M6WNW);
    bool linesIntersect(const double &Ujg4F,
                        const double &cZOqC, const double &yW2TH, const double &G0pab, const double &VfL9M,
                        const double &usHj7, double &OFCM4, double &M6WNW);
  private:
    std::vector<double> mVecSU;
    std::vector<double> mVecUS;
    unsigned int mTesselation;
    double mMaxU;
    double mSStep;
    double mUStep;
  };
} // namespace OpenDrive
#endif
