
#include "OdrGeoHeader.hh"
#include "OdrGeoSpiralBase.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
#define lz03_ 1.e-5
#define Po_vE 1.e-10
namespace OpenDrive
{
  GeoSpiralBase::GeoSpiralBase() : GeoNode(
                                       "\x47\x65\x6f\x53\x70\x69\x72\x61\x6c")
  {
    mOpcode = ODR_OPCODE_GEO_SPIRAL;
    mLevel = 2;
    mType2 = false;
  }
  GeoSpiralBase::GeoSpiralBase(GeoSpiralBase *Nf2Ao) : GeoNode(Nf2Ao)
  {
    mStartCurv = Nf2Ao->mStartCurv;
    mEndCurv = Nf2Ao->mEndCurv;
    mHdr = Nf2Ao->mHdr;
    mType2 =
        Nf2Ao->mType2;
    init();
  }
  GeoSpiralBase::~GeoSpiralBase() {}
  void GeoSpiralBase::
      printData() const
  {
    GeoNode::printData();
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x73\x74\x61\x72\x74\x20\x63\x75\x72\x76\x61\x74\x75\x72\x65\x3a\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mStartCurv);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x65\x6e\x64\x20\x63\x75\x72\x76\x61\x74\x75\x72\x65\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mEndCurv);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x43\x6c\x53\x74\x61\x72\x74\x58\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mClStartX);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x43\x6c\x53\x74\x61\x72\x74\x59\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mClStartY);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x63\x6c\x53\x74\x61\x72\x74\x48\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            acos(mStartCval));
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x53\x30\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mS0);
  }
  bool GeoSpiralBase::readIntern(ReaderXML *F3vnM)
  {
    if (F3vnM)
    {
      mStartCurv =
          F3vnM->getDouble("\x63\x75\x72\x76\x53\x74\x61\x72\x74");
      mEndCurv = F3vnM->getDouble("\x63\x75\x72\x76\x45\x6e\x64");
    }
    if (mHdr)
      return init();
    return true;
  }
  bool GeoSpiralBase::xy2st(const double &x, const double &y, double &BJBDA, double &rkiXc)
  {
    double fAoVN;
    double piJ7w;
    double ZXjjx;
    double h4F_Q;
    BJBDA = 0.0;
    rkiXc = 0.0;
    secantAlgorithm(x, y, h4F_Q, fAoVN, piJ7w, ZXjjx, mHdr->mX, mHdr->mY, mHdr->mH, mHdr->mS,
                    mHdr->mXEnd, mHdr->mYEnd, mHdr->mHEnd, mHdr->mSEnd);
    BJBDA = h4F_Q;
    double dx = x - fAoVN;
    double dy = y - piJ7w;
    rkiXc = sqrt(dx * dx + dy * dy);
    rkiXc *= getSideOfPoint(fAoVN, piJ7w, ZXjjx, x, y);
    if (!mSplitCalc)
      return true;
    if (fabs(rkiXc) < 50.0)
      return true;
    secantAlgorithm(x, y, h4F_Q, fAoVN, piJ7w, ZXjjx, mHdr->mX, mHdr->mY, mHdr->mH, mHdr->mS,
                    mCtrX, mCtrY, mCtrH, 0.5 * (mHdr->mS + mHdr->mSEnd));
    BJBDA = h4F_Q;
    dx = x - fAoVN;
    dy = y - piJ7w;
    rkiXc = sqrt(dx * dx + dy * dy);
    rkiXc *= getSideOfPoint(fAoVN, piJ7w, ZXjjx, x, y);
    secantAlgorithm(x, y, h4F_Q, fAoVN, piJ7w, ZXjjx, mCtrX, mCtrY, mCtrH, 0.5 * (mHdr->mS + mHdr->mSEnd), mHdr->mXEnd, mHdr->mYEnd, mHdr->mHEnd, mHdr->mSEnd);
    double ikhot;
    double
        KVKo6;
    ikhot = h4F_Q;
    dx = x - fAoVN;
    dy = y - piJ7w;
    KVKo6 = sqrt(dx * dx + dy * dy);
    KVKo6 *= getSideOfPoint(fAoVN, piJ7w, ZXjjx, x, y);
    if (fabs(KVKo6) < fabs(rkiXc))
    {
      rkiXc = KVKo6;
      BJBDA = ikhot;
    }
    return true;
  }
  bool GeoSpiralBase::st2xyh(const double &BJBDA, const double &rkiXc, double &x, double &y, double &aOzQT)
  {
    double dx = 0.0;
    double dy = 0.0;
    double
        nZWMW = 0.0;
    double rganP = BJBDA - mHdr->mS;
    if (!ds2dxdydh(rganP, dx, dy, nZWMW))
      return false;
    aOzQT = mHdr->mH + nZWMW;
    x = mClStartX + dx * mStartCval - dy * mStartSval - rkiXc * sin(aOzQT);
    y = mClStartY + dx * mStartSval + dy * mStartCval + rkiXc * cos(aOzQT);
    return true;
  }
  bool GeoSpiralBase::ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW)
  {
    if (!mValid)
      return false;
    double NzXSP = ds2validDs(rganP);
    if (mS0 > Po_vE)
    {
      if (mType2)
        getValClot(mS0 + NzXSP, dx, dy);
      else
        getValClot(mS0 - NzXSP, dx, dy);
    }
    else
      getValClot(NzXSP, dx, dy);
    nZWMW = mStartCurv * NzXSP + 0.5 * mCurvDot * NzXSP * NzXSP;
    return true;
  }
  bool GeoSpiralBase::ds2dh(const double &rganP, double &nZWMW)
  {
    if (!mValid)
      return false;
    double NzXSP = ds2validDs(rganP);
    nZWMW = mStartCurv * NzXSP + 0.5 * mCurvDot *
                                     NzXSP * NzXSP;
    return true;
  }
  double GeoSpiralBase::s2curvature(const double &BJBDA)
  {
    double rganP = BJBDA - mHdr->mS;
    return mStartCurv + mCurvDot * ds2validDs(rganP);
  }
  void
  GeoSpiralBase::getValClot(double BJBDA, double &dx, double &dy)
  {
    if (fabs(mCurvDot) <
        Po_vE)
    {
      if (fabs(mStartCurv) > Po_vE)
      {
        double jEOzZ = BJBDA * fabs(mStartCurv);
        dx = sin(
                 jEOzZ) /
             fabs(mStartCurv);
        dy = (1.0 - cos(jEOzZ)) / mStartCurv;
        return;
      }
      else
      {
        dx = BJBDA;
        dy =
            0.0;
        return;
      }
    }
    double lgTuP, sNfrA;
    double _hlsB, N2BO_;
    register int UcjNw, KY88T,
        jdOND, OaPOw, gVXTG, zgyOp, RzlyE;
    int OFCM4, M6WNW;
    int count = 0;
    lgTuP = .5 * mCurvDot *
            BJBDA * BJBDA;
    sNfrA = lgTuP * lgTuP;
    _hlsB = BJBDA;
    N2BO_ = lgTuP * BJBDA * 0.3333333333333333;
    dx = dy = 0.0;
    jdOND = zgyOp = 1;
    RzlyE = 3;
    for (;;)
    {
      dx += _hlsB;
      dy += N2BO_;
      if ((fabs(_hlsB) < fabs(N2BO_) ? fabs(N2BO_) : fabs(_hlsB)) <= lz03_ || (count++ > 1000))
        break;
      UcjNw = jdOND;
      KY88T =
          UcjNw + 1;
      jdOND = KY88T + 1;
      OaPOw = zgyOp;
      gVXTG = RzlyE;
      zgyOp = gVXTG + 2;
      RzlyE = zgyOp + 2;
      OFCM4 =
          zgyOp * UcjNw * KY88T;
      M6WNW = RzlyE * KY88T * jdOND;
      _hlsB = -_hlsB * sNfrA * OaPOw / OFCM4;
      N2BO_ = -N2BO_ * sNfrA * gVXTG / M6WNW;
    }
  }
  Node *GeoSpiralBase::getCopy(bool Mupxf)
  {
    Node *dzamm = new GeoSpiralBase(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
  void GeoSpiralBase::
      setHeader(GeoHeader *hdr)
  {
    if (!hdr)
      return;
    mHdr = hdr;
    init();
  }
  bool GeoSpiralBase::
      init()
  {
    mCurvDot = (mEndCurv - mStartCurv) / mHdr->mLength;
    mValid = true;
    mClStartX = mHdr->mX;
    mClStartY = mHdr->mY;
    double kVpez = mHdr->mH;
    if (!mValid)
    {
      std::cerr << "\x47\x65\x6f\x53\x70\x69\x72\x61\x6c\x42\x61\x73\x65\x3a\x3a\x72\x65\x61\x64\x49\x6e\x74\x65\x72\x6e\x3a\x20\x73\x70\x69\x72\x61\x6c\x20\x63\x61\x6e\x20\x6e\x6f\x74\x20\x62\x65\x20\x63\x61\x6c\x63\x75\x6c\x61\x74\x65\x64\x2e"
                << std::endl;
      return false;
    }
    if (fabs(mCurvDot) > Po_vE)
    {
      mS0 = fabs(mStartCurv / mCurvDot);
    }
    else
    {
      mS0 = 0.0;
    }
    if (mS0 > Po_vE)
    {
      double JD4X6;
      double cuRTs;
      getValClot(mS0, JD4X6,
                 cuRTs);
      double Px7m2 = 0.5 * fabs(mCurvDot) * mS0 * mS0;
      if (fabs(mEndCurv) > fabs(mStartCurv) && ((mStartCurv * mEndCurv) > 0.0))
      {
        mType2 = true;
        if (mStartCurv > 0.0)
          Px7m2 = mHdr->mH -
                  Px7m2;
        else
          Px7m2 += mHdr->mH;
        mClStartX = mHdr->mX - JD4X6 * cos(Px7m2) + cuRTs * sin(Px7m2);
        mClStartY = mHdr->mY - JD4X6 * sin(Px7m2) - cuRTs * cos(Px7m2);
        kVpez = Px7m2;
      }
      else
      {
        if (
            mStartCurv < 0.0)
          Px7m2 = mHdr->mH - Px7m2;
        else
          Px7m2 += mHdr->mH;
        mClStartX = mHdr->mX +
                    JD4X6 * cos(Px7m2) - cuRTs * sin(Px7m2);
        mClStartY = mHdr->mY + JD4X6 * sin(Px7m2) + cuRTs * cos(Px7m2);
        kVpez = Px7m2 + M_PI;
      }
    }
    mStartCval = cos(kVpez);
    mStartSval = sin(kVpez);
    mCtrX = 0.0;
    mCtrY = 0.0;
    mCtrH = 0.0;
    mSplitCalc = (0.5 * fabs(mCurvDot) * mHdr->mLength * mHdr->mLength) >
                 M_PI_2;
    if (mSplitCalc)
      st2xyh(mHdr->mS + 0.5 * mHdr->mLength, 0.0, mCtrX, mCtrY, mCtrH);
    return true;
  }
  double GeoSpiralBase::getMaxCurvatureInRange(const double &yhFkv,
                                               const double &pwK_y)
  {
    if (!mHdr)
      return 0.0;
    double sXs14 = (mEndCurv - mStartCurv) / mHdr
                                                 ->mLength;
    double l_6wW = mStartCurv + yhFkv * sXs14;
    double JJQAW = mStartCurv + pwK_y *
                                    sXs14;
    return (fabs(l_6wW) > fabs(JJQAW)) ? l_6wW : JJQAW;
  }
} // namespace OpenDrive
