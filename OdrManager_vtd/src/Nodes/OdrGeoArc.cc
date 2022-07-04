
#include "OdrGeoHeader.hh"
#include "OdrGeoArc.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive
{
  GeoArc::GeoArc() : GeoNode("\x47\x65\x6f\x41\x72\x63")
  {
    mOpcode = ODR_OPCODE_GEO_ARC;
    mLevel = 2;
  }
  GeoArc::~GeoArc() {}
  GeoArc::GeoArc(GeoArc *Nf2Ao) : GeoNode(Nf2Ao)
  {
    mCurv = Nf2Ao->mCurv;
    mHdr = Nf2Ao->mHdr;
    init();
  }
  void GeoArc::
      printData() const
  {
    GeoNode::printData();
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x63\x75\x72\x76\x61\x74\x75\x72\x65\x3a\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mCurv);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x63\x74\x72\x58\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mCtrX);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x63\x74\x72\x59\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mCtrY);
  }
  bool GeoArc::readIntern(ReaderXML *F3vnM)
  {
    if (F3vnM)
    {
      mCurv = F3vnM->getDouble("\x63\x75\x72\x76\x61\x74\x75\x72\x65");
    }
    if (mHdr)
      return init();
    return true;
  }
  bool GeoArc::ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW)
  {
    double NzXSP = ds2validDs(rganP);
    nZWMW = NzXSP * mCurv;
    dx = (sin(mHdr->mH + nZWMW) -
          mStartSval) /
         mCurv;
    dy = -(cos(mHdr->mH + nZWMW) - mStartCval) / mCurv;
    return true;
  }
  bool
  GeoArc::ds2dh(const double &rganP, double &nZWMW)
  {
    double NzXSP = ds2validDs(rganP);
    nZWMW = NzXSP * mCurv;
    return true;
  }
  double GeoArc::s2curvature(const double &) { return mCurv; }
  Node *GeoArc::getCopy(bool Mupxf)
  {
    Node *dzamm = new GeoArc(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
  void GeoArc::setHeader(GeoHeader *hdr)
  {
    if (!hdr)
      return;
    mHdr = hdr;
    init();
  }
  bool GeoArc::xy2st(const double &x, const double &y, double &BJBDA, double &rkiXc)
  {
    double dx = x - mCtrX;
    double dy = y - mCtrY;
    double dist = sqrt(dx * dx +
                       dy * dy);
    rkiXc = dist - fabs(1.0 / mCurv);
    if (fabs(rkiXc) < 1.0e-6)
      rkiXc = 0.0;
    else if (mCurv >
             0.0)
      rkiXc *= -1.0;
#ifdef ttGYq
    if (dist < 1.0e-6)
    {
      BJBDA = mHdr->mS;
      return true;
    }
    dx /= dist;
    dy /= dist;
    double V3Jzr = (dx * (mHdr->mX - mCtrX) + dy * (mHdr->mY - mCtrY)) * fabs(mCurv);
    double CTbau = (dx * (mHdr->mXEnd -
                          mCtrX) +
                    dy * (mHdr->mYEnd - mCtrY)) *
                   fabs(mCurv);
    if (V3Jzr > CTbau)
    {
      double LzkrF = acos(
          V3Jzr);
      BJBDA = mHdr->mS + fabs(LzkrF / mCurv);
    }
    else
    {
      double LzkrF = acos(CTbau);
      BJBDA =
          mHdr->mSEnd - fabs(LzkrF / mCurv);
    }
    if (BJBDA < mHdr->mS || BJBDA > mHdr->mSEnd)
    {
      fprintf(
          stderr,
          "\x47\x65\x6f\x41\x72\x63\x3a\x3a\x78\x79\x32\x73\x74\x3a\x20\x65\x72\x72\x6f\x72\x20\x69\x6e\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x63\x61\x6c\x63\x75\x6c\x61\x74\x69\x6f\x6e\x2c\x20\x73\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x76\x61\x6c\x69\x64\x20\x72\x61\x6e\x67\x65\x20\x5b\x25\x2e\x33\x6c\x66\x3b\x25\x2e\x33\x6c\x66\x5d"
          "\n",
          BJBDA, mHdr->mS, mHdr->mSEnd);
      return false;
    }
#else
    double u54Cu = atan2(mHdr->mY - mCtrY, mHdr->mX - mCtrX);
    double REDlx = atan2(dy, dx);
    if ((
            mCurv < 0.0) &&
        (REDlx > u54Cu))
      REDlx -= 2.0 * M_PI;
    else if ((mCurv > 0.0) && (REDlx < u54Cu))
      REDlx += 2.0 * M_PI;
    BJBDA = fabs((REDlx - u54Cu) / mCurv) + mHdr->mS;
    if (fabs(BJBDA) < 1.0e-6)
      BJBDA = 0.0;
    if (fabs(BJBDA - mHdr->mSEnd) < 1.0e-6)
      BJBDA = mHdr->mSEnd;
    if (0)
    {
      double dx;
      double dy;
      double nZWMW;
      double rganP = mHdr->mSEnd - mHdr->mS;
      ds2dxdydh(rganP, dx, dy,
                nZWMW);
      fprintf(stderr,
              "\x47\x65\x6f\x41\x72\x63\x3a\x3a\x78\x79\x32\x73\x74\x3a\x20\x78\x52\x69\x67\x68\x74\x20\x3d\x20\x25\x2e\x31\x36\x65\x2c\x20\x79\x52\x69\x67\x68\x74\x20\x3d\x20\x25\x2e\x31\x36\x65\x2c\x20\x64\x73\x20\x3d\x20\x25\x2e\x31\x36\x65\x2c\x20\x68\x52\x69\x67\x68\x74\x20\x3d\x20\x25\x2e\x31\x36\x65"
              "\n",
              mHdr->mX + dx, mHdr->mY + dy, rganP, mHdr->mH + nZWMW);
    }
    if (BJBDA < mHdr->mS || BJBDA > mHdr->mSEnd)
    {
      return false;
    }
#endif
    return true;
  }
  bool GeoArc::containsPos(const double &x, const double &y)
  {
    double
        jEOzZ = fabs(mHdr->mLength * mCurv);
    return pointIsInSection(x, y, mHdr->mX, mHdr->mY,
                            mHdr->mH, mHdr->mS, mHdr->mXEnd, mHdr->mYEnd, mHdr->mHEnd, mHdr->mSEnd, (jEOzZ < M_PI) ? 1 : 2);
  }
  bool GeoArc::init()
  {
    double tyz2F = 1.0 / mCurv;
    double EpBr1 = mHdr->mH + M_PI_2;
    mCtrX = mHdr->mX + tyz2F * cos(EpBr1);
    mCtrY = mHdr->mY + tyz2F * sin(EpBr1);
    return true;
  }
  double GeoArc::getMaxCurvatureInRange(const double &, const double &)
  {
    return mCurv;
  }
} // namespace OpenDrive
