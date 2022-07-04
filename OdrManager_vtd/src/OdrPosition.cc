
#include <math.h>
#include "OdrGeoHeader.hh"
#include "OdrElevation.hh"
#include "OdrCrossfall.hh"
#include "OdrSuperelevation.hh"
#include "OdrLaneSection.hh"
#include "OdrRoadLink.hh"
#include "OdrSignal.hh"
#include "OdrPosition.hh"
#include "OdrPath.hh"
#include "OdrRoadMark.hh"
#include <iostream>
#include <string>
namespace OpenDrive
{
  double Position::getInertialDist(const Position &_3xSj, const Position &yrOeM) { return Coord::getDist(_3xSj.getInertialPos(), yrOeM.getInertialPos()); }
  Position::Position() : mCurvature(0.0), mLaneCurvature(0.0),
                         mLaneCurvatureDot(0.0), mLaneCurvatureVert(0.0), mLaneCurvatureVertDot(0.0),
                         mLaneDeltaHdg(0.0), mTrackWidth(0.0), mLaneWidth(0.0), mRoadMark(0), mDirChanged(false),
                         mPathS(0.0), mPath(0)
  {
    mRoadQuery = new RoadQuery("\x52\x6f\x61\x64\x51\x75\x65\x72\x79");
    mInertialPos.init();
    mLanePos.init();
    mLaneList.clear();
    mJuncInfList.clear();
  }
  Position::Position(const Position &) : mCurvature(0.0), mLaneCurvature(0.0), mLaneCurvatureDot(0.0), mLaneCurvatureVert(0.0),
                                         mLaneCurvatureVertDot(0.0), mLaneDeltaHdg(0.0), mTrackWidth(0.0), mLaneWidth(0.0),
                                         mRoadMark(0), mDirChanged(false), mPathS(0.0), mPath(0)
  {
    mRoadQuery = new RoadQuery("\x52\x6f\x61\x64\x51\x75\x65\x72\x79");
    mInertialPos.init();
    mGeoPos.init();
    mLanePos.init();
    mLaneList.clear();
    mJuncInfList.clear();
  }
  Position::~Position() { delete mRoadQuery; }
  void Position::operator=(const Position &Nf2Ao)
  {
    mInertialPos = Nf2Ao.getInertialPos();
    mLanePos = Nf2Ao.getLanePos();
    mGeoPos = Nf2Ao.getGeoPos();
    mRoadQuery->fillHistory(Nf2Ao.getHistory());
    mLaneList = Nf2Ao.getLaneList();
    mJuncInfList = Nf2Ao.getJuncInfList();
  }
  void Position::operator=(
      const Coord &Nf2Ao) { mInertialPos = Nf2Ao; }
  void Position::operator=(const GeoCoord &
                               Nf2Ao) { mGeoPos = Nf2Ao; }
  void Position::operator+=(const Coord &Nf2Ao) { mInertialPos += Nf2Ao; }
  void Position::operator+=(const GeoCoord &Nf2Ao) { mGeoPos += Nf2Ao; }
  void
  Position::operator+=(const TrackCoord &Nf2Ao) { mLanePos += Nf2Ao; }
  void Position::
  operator+=(const LaneCoord &Nf2Ao) { mLanePos += Nf2Ao; }
  const TrackCoord &Position::
      getTrackPos() const { return mLanePos; }
  const LaneCoord &Position::getLanePos() const
  {
    return mLanePos;
  }
  const Coord &Position::getInertialPos() const { return mInertialPos; }
  const GeoCoord &Position::getGeoPos() const { return mGeoPos; }
  const Coord &Position ::getFootPoint() const { return mFootPoint; }
  void Position::setPos(const TrackCoord &
                            value) { mLanePos = value; }
  void Position::setTrackPos(const int &id, const double &BJBDA, const double &rkiXc)
  {
    mLanePos.setTrackId(id);
    mLanePos.setS(BJBDA);
    mLanePos.setT(rkiXc);
  }
  void Position::setTrackPos(const std::string &id, const double &BJBDA,
                             const double &rkiXc)
  {
    mLanePos.setTrackId(id);
    mLanePos.setS(BJBDA);
    mLanePos.setT(
        rkiXc);
  }
  void Position::setTrackPos(const TrackCoord &value) { mLanePos = value; }
  void
  Position::setPos(const LaneCoord &value) { mLanePos = value; }
  void Position::
      setLanePos(const int &LqMUn, const int &laneId, const double &BJBDA, const double &offset)
  {
    mLanePos.setTrackId(LqMUn);
    mLanePos.setLaneId(laneId);
    mLanePos.setS(
        BJBDA);
    mLanePos.setOffset(offset);
  }
  void Position::setLanePos(const std::string &
                                LqMUn,
                            const int &laneId, const double &BJBDA, const double &offset)
  {
    mLanePos.setTrackId(LqMUn);
    mLanePos.setLaneId(laneId);
    mLanePos.setS(BJBDA);
    mLanePos.setOffset(offset);
  }
  void Position::setLanePos(const LaneCoord &value) { mLanePos = value; }
  
  void Position::setPos(const Coord &value) { mInertialPos = value; }

  void Position::setInertialPos(const double &x, const double &y, const double &hkK5C)
  {
    mInertialPos = Coord(x, y, hkK5C);
  }

  void Position::setPos(const GeoCoord &value)
  {
    mGeoPos = value;
  }
  void Position::setGeoPos(const double &auc9o, const double &smK7U,
                           const double &hkK5C) 
    { mGeoPos = GeoCoord(auc9o, smK7U, hkK5C); }

  int Position::track2inertial()
  {
    mCurvature = 0.0;
    int result = mRoadQuery->track2inertial(mInertialPos, mLanePos);
    if (result == RoadQuery::RESULT_EXCEEDS_ROAD)
      mLanePos.setS(mRoadQuery->getMaxTrackPos());
    else
      mCurvature = mRoadQuery->getCurvature();
    return result;
  }
  int Position::track2lane() { return mRoadQuery->track2lane(mLanePos); }
  int Position::track2curvature()
  {
    mCurvature = 0.0;
    int result = mRoadQuery->track2curvature(mLanePos);
    if (result == RoadQuery::RESULT_ON_ROAD)
      mCurvature =
          mRoadQuery->getCurvature();
    return result;
  }
  int Position::track2inertialAngDotCrossSec()
  {
    Coord mZvni;
    mCrossSection.clear();
    int result =
        mRoadQuery->track2inertialAngDotCrossSec(mLanePos, mInertialPos, mZvni,
                                                 mCrossSection);
    mTrackAngles = mInertialPos;
    mDhDs = mZvni.getH();
    mDpDs = mZvni.getP();
    mDrDs = mZvni.getR();
    mDzDs = -tan(mTrackAngles.getP()) + mLanePos.getT() * mDrDs * cos(mTrackAngles.getR());
    mDzDt = sin(mTrackAngles.getR());
    mD2zDs = mZvni.getZ();
    if (result == RoadQuery::RESULT_ON_ROAD)
      mCurvature = mRoadQuery->getCurvature();
    else
      mCurvature = 0.0;
    return result;
  }
  int Position::inertial2track(const double &Yrh_j)
  {
    mCurvature = 0.0;
    int result = mRoadQuery->inertial2track(mInertialPos, mLanePos, Yrh_j);
    if (result == RoadQuery::RESULT_ON_ROAD)
    {
      mCurvature = mRoadQuery->getCurvature();
      mFootPoint = mRoadQuery->getFootPoint();
    }
    return result;
  }
  int Position::lane2track()
  {
    int result = mRoadQuery->lane2track(mLanePos);
    if (result == RoadQuery::
                      RESULT_ON_ROAD)
    {
      double jwOMF;
      result = mRoadQuery->getTrackHeading(mLanePos, jwOMF);
      if (result == RoadQuery::RESULT_ON_ROAD)
        mFootPoint.setH(jwOMF);
    }
    return result;
  }
  int Position::lane2inertial()
  {
    int result = mRoadQuery->lane2track(mLanePos);
    mLaneDeltaHdg = mRoadQuery->getDeltaLaneDir();
    if (result == RoadQuery::RESULT_ON_ROAD)
    {
      result = mRoadQuery->track2inertial(mInertialPos, mLanePos);
      if (result == RoadQuery ::RESULT_ON_ROAD)
      {
        mCurvature = mRoadQuery->getCurvature();
        mInertialPos.setH(
            mInertialPos.getH() + mLaneDeltaHdg);
        mInertialPos.setZ(mInertialPos.getZ() +
                          mRoadQuery->getLaneHeight());
      }
    }
    return result;
  }
  int Position::inertial2lane(bool
                                  ce0Qj,
                              bool lAdDx)
  {
    mCurvature = 0.0;
    mLaneDeltaHdg = 0.0;
    mRoadQuery->setValidLaneTypes(ce0Qj, lAdDx);
    int result = mRoadQuery->inertial2lane(mInertialPos, mLanePos);
    if (
        result == RoadQuery::RESULT_ON_ROAD)
    {
      mLaneWidth = mRoadQuery->getLaneWidth();
      mCurvature = mRoadQuery->getCurvature();
      mFootPoint = mRoadQuery->getFootPoint();
      mLaneDeltaHdg = mRoadQuery->getDeltaLaneDir();
    }
    return result;
  }
  int Position::
      inertial2lane(int LqMUn)
  {
    mCurvature = 0.0;
    mLaneDeltaHdg = 0.0;
    std::vector<RoadHeader
                    *>
        LOM4N;
    RoadHeader *wR6_y = mRoadQuery->trackId2Node(LqMUn);
    if (!wR6_y)
      return RoadQuery::RESULT_ERROR;
    LOM4N.push_back(wR6_y);
    int result = mRoadQuery->inertial2lane(mInertialPos, mLanePos, LOM4N, false);
    if (result == RoadQuery::
                      RESULT_ON_ROAD)
    {
      mLaneWidth = mRoadQuery->getLaneWidth();
      mCurvature = mRoadQuery->getCurvature();
      mFootPoint = mRoadQuery->getFootPoint();
      mLaneDeltaHdg = mRoadQuery->getDeltaLaneDir();
    }
    return result;
  }
  int Position::inertial2lane(const std::string &
                                  LqMUn)
  {
    mCurvature = 0.0;
    mLaneDeltaHdg = 0.0;
    std::vector<RoadHeader *> LOM4N;
    RoadHeader
        *wR6_y = mRoadQuery->trackId2Node(LqMUn);
    if (!wR6_y)
      return RoadQuery::RESULT_ERROR;
    LOM4N.push_back(wR6_y);
    int result = mRoadQuery->inertial2lane(mInertialPos,
                                           mLanePos, LOM4N, false);
    if (result == RoadQuery::RESULT_ON_ROAD)
    {
      mLaneWidth =
          mRoadQuery->getLaneWidth();
      mCurvature = mRoadQuery->getCurvature();
      mFootPoint =
          mRoadQuery->getFootPoint();
      mLaneDeltaHdg = mRoadQuery->getDeltaLaneDir();
    }
    return result;
  }
  int Position::inertial2pathLane(const double &Yrh_j)
  {
    if (!mPath)
      return RoadQuery::RESULT_NOT_ON_ROAD;
    mCurvature = 0.0;
    mLaneDeltaHdg = 0.0;
    std::vector<
        RoadHeader *>
        LOM4N;
    mPath->getRoads(LOM4N, Yrh_j);
    int result = mRoadQuery->inertial2lane(mInertialPos, mLanePos, LOM4N);
    if (result == RoadQuery::RESULT_ON_ROAD)
    {
      if (mPath->pos2s(mLanePos, Yrh_j))
      {
        mLaneWidth = mRoadQuery->getLaneWidth();
        mCurvature = mRoadQuery->getCurvature();
        mFootPoint = mRoadQuery->getFootPoint();
        mLaneDeltaHdg = mRoadQuery->getDeltaLaneDir();
      }
      return RoadQuery::RESULT_ON_ROAD;
    }
    return RoadQuery::RESULT_NOT_ON_ROAD;
  }
  int Position::inertial2laneList(bool dZZZn, bool ce0Qj, bool lAdDx)
  {
    mCurvature = 0.0;
    mRoadQuery->setValidLaneTypes(ce0Qj, lAdDx);
    return mRoadQuery->inertial2lane(mInertialPos, mLaneList, dZZZn);
  }
  int Position::
      inertial2geo() { return mRoadQuery->inertial2geo(mInertialPos, mGeoPos); }
  int Position::geo2inertial() { return mRoadQuery->geo2inertial(mGeoPos, mInertialPos); }
  const LaneCoord::LaneVec &Position::getLaneList() const { return mLaneList; }
  const RoadQuery::JuncInfVec &Position::getJuncInfList() const { return mJuncInfList; }
  void
  Position::print(int NS8oU)
  {
    std::string LnWUO;
    for (int i = 0; i < NS8oU; ++i)
      LnWUO = LnWUO + ((char)(0x1bf1 + 2337 - 0x24f2));
    std::cerr << LnWUO;
    mLanePos.print();
    std::cerr << LnWUO;
    mInertialPos.print();
    std::cerr << LnWUO << "\x63\x75\x72\x76\x61\x74\x75\x72\x65\x20\x3d\x20" << mCurvature << std::endl;
  }
  const double &Position::getCurvature() const { return mCurvature; }
  const double &Position::getLaneCurvature() const { return mLaneCurvature; }
  const double &Position::getLaneCurvatureDot() const { return mLaneCurvatureDot; }
  const double &Position::getLaneCurvatureVert() const { return mLaneCurvatureVert; }
  const double &Position::getLaneCurvatureVertDot() const { return mLaneCurvatureVertDot; }
  const double &
  Position::getDeltaLaneDir() const { return mLaneDeltaHdg; }
  const double &Position::getTrackLen(int LqMUn)
  {
    Coord Tlj7t;
    TrackCoord HQGFE(LqMUn, 0.0, 0.0);
    mRoadQuery->track2inertial(Tlj7t, HQGFE);
    return mRoadQuery->getMaxTrackPos();
  }
  const double &Position::getTrackLen(const std::string &LqMUn)
  {
    Coord Tlj7t;
    TrackCoord HQGFE(
        LqMUn, 0.0, 0.0);
    mRoadQuery->track2inertial(Tlj7t, HQGFE);
    return mRoadQuery->getMaxTrackPos();
  }
  void Position::calcTrackWidth() { mTrackWidth = mRoadQuery->getTrackWidth(mLanePos); }
  const double &Position::getTrackWidth() const { return mTrackWidth; }
  void Position::calcTrackAngles() { mRoadQuery->getTrackAngles(mLanePos, mTrackAngles); }
  const Coord &Position::getTrackAngles() const { return mTrackAngles; }
  void Position::calcTrackAnglesDot(bool V_5CG)
  {
    if (V_5CG)
      mTrackAngles = mInertialPos;
    else
      mRoadQuery->getTrackAngles(mLanePos, mTrackAngles);
    Coord mZvni;
    mRoadQuery->getTrackAnglesDot(mLanePos, mZvni);
    mDhDs = mZvni.getH();
    mDpDs = mZvni.getP();
    mDrDs = mZvni.getR();
    mDzDs = -tan(mTrackAngles.getP()) + mLanePos.getT() * mDrDs * cos(mTrackAngles.getR());
    mDzDt = sin(mTrackAngles.getR());
    mD2zDs = mZvni.getZ();
  }
  int Position::lane2laneWidth()
  {
    int result = mRoadQuery->lane2laneWidth(mLanePos);
    if (result == RoadQuery::RESULT_ON_ROAD)
    {
      mLaneWidth =
          mRoadQuery->getLaneWidth();
      mRoadMark = mRoadQuery->getRoadMark();
    }
    else
    {
      mLaneWidth = 0.0;
      mRoadMark = 0;
    }
    return result;
  }
  int Position::lane2roadMark()
  {
    int result = mRoadQuery->lane2roadMark(mLanePos);
    if (result == RoadQuery::RESULT_ON_ROAD)
      mRoadMark = mRoadQuery->getRoadMark();
    else
      mRoadMark = 0;
    return result;
  }
  const double& Position::getLaneWidth() const { return mLaneWidth; }
  int Position::lane2laneCurvature()
  {
    int result = mRoadQuery->lane2curvature(mLanePos);
    if (result == RoadQuery::RESULT_ON_ROAD)
      mRoadQuery->getLaneCurvature(mLaneCurvature, mLaneCurvatureDot, mLaneCurvatureVert, mLaneCurvatureVertDot);
    else
    {
      mLaneCurvature = 0.0;
      mLaneCurvatureDot = 0.0;
      mLaneCurvatureVert = 0.0;
      mLaneCurvatureVertDot = 0.0;
    }
    return result;
  }
  const unsigned short &Position::getRoadMark() const { return mRoadMark; }
  int Position::laneBorder2inertial(bool GirtK)
  {
    int result = lane2laneWidth();
    if (result != RoadQuery::RESULT_ON_ROAD)
      return result;
    double wzKzI = GirtK ? 1.0 : -1.0;
    if (mLanePos.getLaneId() > 0)
      mLanePos.setOffset(wzKzI * 0.5 * mLaneWidth);
    else
      mLanePos.setOffset(-wzKzI * 0.5 * mLaneWidth);
    return lane2inertial();
  }
  int Position::track2validTrack()
  {
    int z6riB = mLanePos.getTrackId();
    mDirChanged = false;
    if (mRoadQuery->track2validTrack(mLanePos, mDirChanged) != RoadQuery::RESULT_ON_ROAD)
      return RoadQuery::RESULT_ERROR;
    if (mLanePos.getTrackId() == z6riB)
      return RoadQuery::RESULT_ON_ROAD;
    if (mDirChanged)
      mLanePos.setT(-mLanePos.getT());
    int result = track2lane();
    if (result != RoadQuery::RESULT_ON_ROAD)
      return result;
    return track2curvature();
  }
  int Position::lane2validLane()
  {
    int z6riB = mLanePos.getTrackId();
    mDirChanged = false;
    if (mRoadQuery->lane2validLane(mLanePos,
                                   mDirChanged) != RoadQuery::RESULT_ON_ROAD)
      return RoadQuery::RESULT_ERROR;
    if (
        mLanePos.getTrackId() == z6riB)
      return RoadQuery::RESULT_ON_ROAD;
    if (mDirChanged)
      mLanePos.setOffset(-mLanePos.getOffset());
    int result = lane2track();
    if (result !=
        RoadQuery::RESULT_ON_ROAD)
      return result;
    return track2curvature();
  }
  LaneCoord
  Position::getRelativePos(const LaneCoord &Hv9fs)
  {
    double QXmP6;
    double jYJNb;
    int
        F6MiM;
    int EqrOt;
    double Mmn3n;
    if (Hv9fs.getTrackId() == mLanePos.getTrackId())
    {
      EqrOt = Hv9fs.getTrackId() - mLanePos.getTrackId();
      QXmP6 = Hv9fs.getS() - mLanePos.getS();
      jYJNb = Hv9fs.getT() - mLanePos.getT();
      F6MiM = Hv9fs.getLaneId() - mLanePos.getLaneId();
      Mmn3n = Hv9fs.getOffset() - mLanePos.getOffset();
      LaneCoord qOvzn(EqrOt, F6MiM, QXmP6,
                      Mmn3n);
      qOvzn.setT(jYJNb);
      return qOvzn;
    }
    Path zbHKq(
        "\x72\x65\x6c\x61\x74\x69\x76\x65\x20\x50\x6f\x73");
    zbHKq.calcFromTrackPositions(mLanePos, Hv9fs);
    bool GfY2S = zbHKq.sameTrackDir(0.0, zbHKq.getLength());
    int wzKzI =
        GfY2S ? 1 : -1;
    EqrOt = Hv9fs.getTrackId() - mLanePos.getTrackId();
    QXmP6 = zbHKq.getFwdAtS(
                0.0)
                ? zbHKq.getLength()
                : -zbHKq.getLength();
    jYJNb = wzKzI * Hv9fs.getT() - mLanePos.getT();
    F6MiM = wzKzI * Hv9fs.getLaneId() - mLanePos.getLaneId();
    Mmn3n = wzKzI * Hv9fs.getOffset() - mLanePos.getOffset();
    LaneCoord qOvzn(EqrOt, F6MiM, QXmP6, Mmn3n);
    qOvzn.setT(jYJNb);
    return qOvzn;
  }
  bool Position::dirChanged() const { return mDirChanged; }
  const double &Position::getDhDs() const { return mDhDs; }
  const double &Position::
      getDpDs() const { return mDpDs; }
  const double &Position::getDrDs() const
  {
    return mDrDs;
  }
  const double &Position::getDzDs() const { return mDzDs; }
  const double &Position::
      getD2zDs() const { return mD2zDs; }
  const double &Position::getDzDt() const { return mDzDt; }
  bool Position::calcCrossSection()
  {
    mCrossSection.clear();
    bool retVal =
        mRoadQuery->getCrossSection(mLanePos, mCrossSection);
    return retVal;
  }
  int Position ::getCrossSectionSize() { return mCrossSection.size(); }
  bool Position::
      getCrossSectionLaneInfo(const int &id, int &laneId, int &zgDYE, double &qfK7D)
  {
    if (id < 0 || id >= (int)(mCrossSection.size()))
      return false;
    laneId = mCrossSection.at(id).id;
    zgDYE = mCrossSection.at(id).type;
    qfK7D = mCrossSection.at(id).width;
    return true;
  }
  bool Position::collectSignals(bool JN4Nc, const double &F5cuE)
  {
    mResultVec.clear();
    bool retVal = mRoadQuery->getSignals(mLanePos, JN4Nc, F5cuE, mResultVec);
    return retVal;
  }
  bool Position::collectObjects(bool JN4Nc, const double &F5cuE)
  {
    mResultVec.clear();
    bool retVal = mRoadQuery->getObjects(mLanePos, JN4Nc, F5cuE, mResultVec);
    return retVal;
  }
  int Position::getCollectionSize() { return mResultVec.size(); }
  bool
  Position::getCollectionInfo(const int &id, double &dist, Node *&node)
  {
    if (id < 0 || id >= (int)(mResultVec.size()))
      return false;
    dist = mResultVec.at(id).dist;
    node = mResultVec
               .at(id)
               .node;
    return true;
  }
  void Position::footPoint2inertial() { mInertialPos = mFootPoint; }

  RoadHeader *Position::getQueriedRoadHeader() { 
    return reinterpret_cast<RoadHeader *>(mRoadQuery->getQueriedNodeOfType(ODR_OPCODE_ROAD_HEADER)); 
  }

  LaneSection *Position::getQueriedLaneSection() { 
    return reinterpret_cast<LaneSection *>(mRoadQuery->getQueriedNodeOfType(ODR_OPCODE_LANE_SECTION));
  }

  Lane *Position::getQueriedLane() { 
    return reinterpret_cast<Lane *>(mRoadQuery->getQueriedNodeOfType(ODR_OPCODE_LANE)); 
  }

  Elevation *Position::getQueriedElevation() { 
    return reinterpret_cast<Elevation *>(mRoadQuery->getQueriedNodeOfType(ODR_OPCODE_ELEVATION)); 
  }

  Superelevation *Position::getQueriedSuperelevation()
  {
    return reinterpret_cast<Superelevation *>(mRoadQuery->getQueriedNodeOfType(
        ODR_OPCODE_SUPERELEVATION));
  }

  RoadMark *Position::getQueriedRoadMark() { 
    return reinterpret_cast<RoadMark *>(mRoadQuery->getQueriedNodeOfType(ODR_OPCODE_LANE_ROAD_MARK)); 
  }

  int Position::getJunctionId()
  {
    RoadHeader *hdr =
        getQueriedRoadHeader();
    if (hdr)
      return hdr->mJuncNo;
    return -1;
  }
  unsigned int
  Position::getLaneType()
  {
    Lane *lane = getQueriedLane();
    if (lane)
      return lane->mType;
    return ODR_LANE_TYPE_NONE;
  }
  bool Position::getMaterial(std::string &o055c, double &jGi0_, double &zJQyy)
  {
    unsigned int code = 0;
    return mRoadQuery->getMaterial(mLanePos,
                                   code, jGi0_, zJQyy, o055c);
  }
  bool Position::getMaterial(unsigned int &code, double &jGi0_, double &zJQyy)
  {
    std::string o055c;
    return mRoadQuery->getMaterial(mLanePos,
                                   code, jGi0_, zJQyy, o055c);
  }
  bool Position::getLaneSpeed(double &kMjGG) { return mRoadQuery->getLaneSpeed(mLanePos, kMjGG); }
  bool Position::intersectCircle(const double &tyz2F) { return mRoadQuery->intersectCircle(mInertialPos, tyz2F, mLaneList); }
  void Position::setPath(Path *japsW) { mPath = japsW; }
  void Position::setPath(Path &
                             japsW) { setPath(&japsW); }
  Path *Position::getPath() { return mPath; }
  int Position::
      track2path()
  {
    if (!mPath)
      return RoadQuery::RESULT_ERROR;
    int result = track2lane();
    if (result != RoadQuery::RESULT_ON_ROAD)
      return result;
    return lane2path();
  }
  int Position::lane2path()
  {
    if (!mPath)
      return RoadQuery::RESULT_ERROR;
    if (!mPath->pos2s(
            mLanePos, mPathS))
      return RoadQuery::RESULT_ERROR;
    mPathS = mPath->getS();
    return RoadQuery::RESULT_ON_ROAD;
  }
  int Position::inertial2path()
  {
    if (!mPath)
      return RoadQuery::RESULT_ERROR;
    int result = inertial2pathLane(mPathS);
    if (result !=
        RoadQuery::RESULT_ON_ROAD)
    {
      result = inertial2pathLane();
      if (result != RoadQuery::
                        RESULT_ON_ROAD)
        return result;
    }
    return lane2path();
  }
  int Position::addPathOffset(
      const double &rganP, const int &xCtky, const double &gmQV0)
  {
    if (!mPath)
      return RoadQuery::RESULT_ERROR;
    if (!mPath->addOffsetPos(rganP, xCtky, gmQV0))
      return RoadQuery::RESULT_ERROR;
    mPathS = mPath->getS();
    setLanePos(mPath->getLanePos());
    return RoadQuery::RESULT_ON_ROAD;
  }
  int Position::getRoadType() { return mRoadQuery
                                    ->getRoadType(mLanePos); }
  int Position::setPathPos(const double &BJBDA)
  {
    if (!mPath)
      return RoadQuery::RESULT_ERROR;
    if (!mPath->setPos(BJBDA))
      return RoadQuery::
          RESULT_ERROR;
    mPathS = mPath->getS();
    setLanePos(mPath->getLanePos());
    return RoadQuery::RESULT_ON_ROAD;
  }
  bool Position::collectJunctionInfo()
  {
    RoadHeader *wR6_y = getQueriedRoadHeader();
    if (!wR6_y)
      return false;
    JuncHeader *jHdr = reinterpret_cast<
        JuncHeader *>(wR6_y->getJunction());
    if (!jHdr)
      return false;
    mRoadQuery->collectJuncInfo(jHdr, mJuncInfList, wR6_y);
    return true;
  }
  int Position::
      getJunctionInfoSize() { return mJuncInfList.size(); }
  bool Position::getJunctionInfo(const unsigned int &hFNbl, JuncHeader *&rQQS4, RoadHeader *&inRoad, RoadHeader *&connRoad, double &turnAngle, int &XWWHl, int &ckJEX)
  {
    if (hFNbl >= mJuncInfList.size())
      return false;
    rQQS4 = mJuncInfList.at(hFNbl).jHdr;
    inRoad = mJuncInfList.at(hFNbl).inRoad;
    connRoad = mJuncInfList.at(hFNbl).connRoad;
    turnAngle = mJuncInfList.at(hFNbl)
                    .turnAngle;
    XWWHl = mJuncInfList.at(hFNbl).minLaneIn;
    ckJEX = mJuncInfList.at(hFNbl).maxLaneIn;
    return true;
  }
  bool Position::getNextJunction(bool JN4Nc, JuncHeader *&jHdr, double &ioAAo)
  {
    RoadHeader *inRoad = 0;
    if (mRoadQuery->getNextJunction(mLanePos,
                                    JN4Nc, jHdr, ioAAo, inRoad))
      return mRoadQuery->collectJuncInfo(jHdr, mJuncInfList,
                                         inRoad);
    return false;
  }
  void Position::setRoadData(RoadData *data) { mRoadQuery->setRoadData(data); }
  void Position::setMaxTrackDist(const double &F5cuE) { mRoadQuery
                                                            ->setMaxTrackDist(F5cuE); }
  void Position::clearHistory() { mRoadQuery->clearHistory(); }
  const HierCoord &Position::getHistory() const
  {
    return mRoadQuery->getHistory();
  }
  void Position::setSurfaceScale(const double &wzKzI) { mRoadQuery->setSurfaceScale(
      wzKzI); }
  int Position::addTrackS(const double &rganP, bool SFH3M)
  {
    if (!SFH3M)
    {
      mLanePos.setS(mLanePos.getS() + rganP);
      return track2validTrack();
    }
    return mRoadQuery->addTrackS(mLanePos, rganP, SFH3M);
  }
  bool Position::inTunnel() { return mRoadQuery->inTunnel(mLanePos); }
  bool Position::onBridge() { return mRoadQuery->onBridge(mLanePos); }
  int Position::addLaneS(const double &rganP) { return mRoadQuery
                                                    ->addLaneS(mLanePos, rganP); }
  void Position::setContactPatchDimension(const double
                                              &length,
                                          const double &width)
  {
    if (!mRoadQuery)
      return;
    mRoadQuery->setContactPatchDimension(length, width);
  }
  void Position::useLaneHeight(bool F744N)
  {
    if (!mRoadQuery)
      return;
    mRoadQuery->useLaneHeight(F744N);
  }
  void Position::
      setZOptimization(bool F744N, const float &AiTnB)
  {
    if (!mRoadQuery)
      return;
    mRoadQuery
        ->setZOptimization(F744N, AiTnB);
  }
  bool Position::isOptimumSolution()
  {
    if (!mRoadQuery)
      return true;
    return mRoadQuery->isOptimumSolution();
  }
} // namespace OpenDrive
