
#include "OdrTrackCoord.hh"
#include <iostream>
namespace OpenDrive
{
  TrackCoord::TrackCoord() { init(); }
  TrackCoord::TrackCoord(
      const int &dKSLT, const double &BJBDA, const double &rkiXc, const double &hkK5C, const double &aOzQT, const double &p, const double &yJG47) : mTrackId(dKSLT), mTrackIdAsString(""), mStringDefined(false), mS(BJBDA), mT(rkiXc), mZ(hkK5C), mH(aOzQT), mP(p), mR(yJG47) {}
  TrackCoord::TrackCoord(const std::string &dKSLT, const double &BJBDA, const double &rkiXc, const double &hkK5C, const double &aOzQT, const double &p, const double &yJG47) : mTrackId(-1), mTrackIdAsString(dKSLT), mStringDefined(true), mS(BJBDA), mT(
                                                                                                                                                                                                                                                           rkiXc),
                                                                                                                                                                               mZ(hkK5C), mH(aOzQT), mP(p), mR(yJG47) {}
  TrackCoord::~TrackCoord() {}
  void
  TrackCoord::operator=(const TrackCoord &Nf2Ao)
  {
    mTrackId = Nf2Ao.getTrackId();
    mTrackIdAsString = Nf2Ao.getTrackIdAsString();
    mStringDefined = Nf2Ao.hasStringId();
    mS = Nf2Ao.getS();
    mT = Nf2Ao.getT();
    mZ = Nf2Ao.getZ();
    mH = Nf2Ao.getH();
    mP = Nf2Ao.getP();
    mR = Nf2Ao.getR();
  }
  void TrackCoord::operator+=(const TrackCoord &Nf2Ao)
  {
    mS += Nf2Ao.getS();
    mT += Nf2Ao.getT();
    mZ += Nf2Ao.getZ();
    mH += Nf2Ao.getH();
    mP += Nf2Ao.getP();
    mR +=
        Nf2Ao.getR();
  }
  const int &TrackCoord::getTrackId() const { return mTrackId; }
  const std ::string &TrackCoord::getTrackIdAsString() const { return mTrackIdAsString; }
  const double &TrackCoord::getS() const { return mS; }
  const double &TrackCoord::getT() const
  {
    return mT;
  }
  const double &TrackCoord::getZ() const { return mZ; }
  const double &
  TrackCoord::getH() const { return mH; }
  const double &TrackCoord::getP() const { return mP; }
  const double &TrackCoord::getR() const { return mR; }
  void TrackCoord::setTrackId(
      const int &value)
  {
    mTrackId = value;
    mStringDefined = false;
  }
  void TrackCoord::
      setTrackId(const std::string &value)
  {
    mTrackIdAsString = value;
    mStringDefined = true;
  }
  void TrackCoord::setS(const double &value) { mS = value; }
  void TrackCoord::setT(const double &value) { mT = value; }
  void TrackCoord::setZ(const double &value) { mZ = value; }
  void
  TrackCoord::setH(const double &value) { mH = value; }
  void TrackCoord::setP(const double &value) { mP = value; }
  void TrackCoord::setR(const double &value) { mR = value; }
  void
  TrackCoord::init()
  {
    mTrackIdAsString.clear();
    mTrackId = 0;
    mStringDefined = false;
    mS =
        0.0;
    mT = 0.0;
    mZ = 0.0;
    mH = 0.0;
    mP = 0.0;
    mR = 0.0;
  }
  void TrackCoord::print() const { std::cerr
                                   << "\x54\x72\x61\x63\x6b\x43\x6f\x6f\x72\x64\x3a\x20\x69\x64\x20\x3d\x20" << mTrackId << "\x20\x28" << mTrackIdAsString << "\x29\x2c\x20\x73\x20\x3d\x20" << mS << "\x2c\x20\x74\x20\x3d\x20" << mT << "\x2c\x20\x7a\x20\x3d\x20" << mZ << "\x2c\x20\x68\x20\x3d\x20" << mH << "\x2c\x20\x70\x20\x3d\x20" << mP << "\x2c\x20\x72\x20\x3d\x20" << mR << std::endl; }
  bool TrackCoord::hasStringId() const
  {
    return mStringDefined;
  }
} // namespace OpenDrive
