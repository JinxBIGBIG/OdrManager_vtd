
#include "OdrLaneCoord.hh"
#include <iostream>
namespace OpenDrive
{
  LaneCoord::LaneCoord() { init(); }
  LaneCoord::LaneCoord(const int &dKSLT, const int &lane, const double &BJBDA, const double &offset) : TrackCoord(
                                                                                                           dKSLT, BJBDA, 0.0),
                                                                                                       mLaneId(lane), mOffset(offset) {}
  LaneCoord::LaneCoord(const std::
                           string &dKSLT,
                       const int &lane, const double &BJBDA, const double &offset) : TrackCoord(dKSLT, BJBDA, 0.0), mLaneId(lane), mOffset(offset) {}
  LaneCoord::~LaneCoord() {}
  void
  LaneCoord::operator=(const LaneCoord &Nf2Ao)
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
    mLaneId = Nf2Ao.getLaneId();
    mOffset = Nf2Ao.getOffset();
  }
  void
  LaneCoord::operator=(const TrackCoord &Nf2Ao)
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
    mLaneId = 0;
    mOffset = 0.0;
  }
  void LaneCoord::operator+=(const TrackCoord &Nf2Ao)
  {
    mTrackId += Nf2Ao.getTrackId();
    mS += Nf2Ao.getS();
    mT += Nf2Ao.getT();
    mZ += Nf2Ao.getZ();
    mH += Nf2Ao.getH();
  }
  void LaneCoord::operator+=(const LaneCoord &
                                 Nf2Ao)
  {
    mTrackId += Nf2Ao.getTrackId();
    mS += Nf2Ao.getS();
    mT += Nf2Ao.getT();
    mZ += Nf2Ao.getZ();
    mH += Nf2Ao.getH();
    mLaneId += Nf2Ao.getLaneId();
    mOffset += Nf2Ao.getOffset();
  }
  const int &LaneCoord::getLaneId() const { return mLaneId; }
  const double &LaneCoord::getOffset() const { return mOffset; }
  void LaneCoord::setLaneId(const int &value)
  {
    mLaneId = value;
  }
  void LaneCoord::setOffset(const double &value) { mOffset = value; }
  void
  LaneCoord::init()
  {
    TrackCoord::init();
    mLaneId = 0;
    mOffset = 0.0;
  }
  void LaneCoord::
      print() const { std::cerr << "\x4c\x61\x6e\x65\x43\x6f\x6f\x72\x64\x3a\x20\x74\x72\x61\x63\x6b\x20\x3d\x20" << mTrackId << "\x20\x28" << mTrackIdAsString << "\x29\x2c\x20\x6c\x61\x6e\x65\x20\x3d\x20" << mLaneId << "\x2c\x20\x6f\x66\x66\x73\x65\x74\x20\x3d\x20" << mOffset << "\x2c\x20\x73\x20\x3d\x20" << mS << "\x2c\x20\x74\x20\x3d\x20" << mT << "\x2c\x20\x7a\x20\x3d\x20" << mZ << "\x2c\x20\x68\x20\x3d\x20" << mH << std::endl; }
} // namespace OpenDrive
