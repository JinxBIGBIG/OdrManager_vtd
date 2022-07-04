
#ifndef _OPENDRIVE_GEO_PARAM_POLY_HH
#define _OPENDRIVE_GEO_PARAM_POLY_HH
#include "OdrGeoNode.hh"
#include <vector>
namespace OpenDrive
{
  class GeoParamPoly : public GeoNode
  {
  public:
    double mAu;
    double mBu;
    double mCu;
    double mDu;
    double mAv;
    double mBv;
    double mCv;
    double mDv;
    double mMaxP;
    bool mUseLength;
    bool mInterpolateHeading;
    double mDeltaHeading;
    double N6H_y;
  public:
    explicit GeoParamPoly();
    explicit GeoParamPoly(GeoParamPoly *Nf2Ao);
    virtual ~GeoParamPoly();
    virtual void printData() const;
    bool readIntern(ReaderXML *
                        F3vnM);
    virtual bool ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW);
    virtual bool ds2dh(const double &rganP, double &nZWMW);
    virtual double
    s2curvature(const double &BJBDA);
    virtual double ds2curvature(const double &rganP);
    virtual Node *getCopy(bool Mupxf = false);
    unsigned int getTesselation() const;
    const double &getMaxP() const;
    virtual void calcPostPrepareData();
    virtual double
    getMaxCurvatureInRange(const double &yhFkv, const double &pwK_y);
  protected:
    virtual bool secantAlgorithm(const double &x, const double &y, double &BJBDA, double &fAoVN,
                                 double &piJ7w, double &ZXjjx, const double &iTz4N, const double &jWKSX, const double &gL0sg, const double &_UmVT, const double &Axkjj, const double &ZHaIX, const double &e1x0Z, const double &WRYw2);
  private:
    double ds2p(const double &rganP);
    double p2ds(const double &rganP);
    void p2dxdy(const double &p, double &dx, double &dy);
    double p2dh(const double &p);
    double getFirstDerivU(const double &p);
    double getFirstDerivV(const double &p);
    double getSecondDerivU(const double &p);
    double getSecondDerivV(const double &p);
    bool linesIntersect(const double &Ujg4F, const double &cZOqC, const double &c_TFD, const double &HaFA5, const double &G0pab, const double &VfL9M, const double &uoAjb, const double &NEudn, double &OFCM4, double &M6WNW);
    bool linesIntersect(const double &Ujg4F, const double &cZOqC, const double &yW2TH, const double &G0pab,
                        const double &VfL9M, const double &usHj7, double &OFCM4, double &M6WNW);
  private:
    std::vector<double> mVecSP;
    std::vector<double> mVecPS;
    unsigned int mTesselation;
    double mSStep;
    double mPStep;
  };
} // namespace OpenDrive
#endif
