
#ifndef _OPENDRIVE_POSITION_HH
#define _OPENDRIVE_POSITION_HH
#include "OdrRoadQuery.hh"
#include "OdrGeoCoord.hh"
#include <vector>
namespace OpenDrive
{
  class Path;
  class RoadData;
  class Position
  {
  public:
    static double getInertialDist(const Position &_3xSj, const Position &yrOeM);
  private:
    LaneCoord mLanePos;
    Coord mInertialPos;
    Coord mFootPoint;
    GeoCoord mGeoPos;
    double mCurvature;
    double mLaneCurvature;
    double mLaneCurvatureDot;
    double mLaneCurvatureVert;
    double mLaneCurvatureVertDot;
    double mLaneDeltaHdg;
    double mTrackWidth;
    Coord mTrackAngles;
    RoadQuery *mRoadQuery;
    double mLaneWidth;
    unsigned short mRoadMark;
    bool mDirChanged;
    double mDzDs;
    double mDzDt;
    double mDhDs;
    double mDpDs;
    double mDrDs;
    double mD2zDs;
    RoadQuery::LaneInfVec mCrossSection;
    RoadQuery::ResultVec mResultVec;
    LaneCoord::LaneVec mLaneList;
    double mPathS;
    Path *mPath;
    RoadQuery::JuncInfVec mJuncInfList;
  public:
    explicit Position();
    Position(const Position &Nf2Ao);
    virtual ~Position();
    void operator=(const Position &Nf2Ao);
    void operator=(const Coord &Nf2Ao);
    void operator=(const GeoCoord &Nf2Ao);
    void operator+=(const Coord &Nf2Ao);
    void operator+=(const GeoCoord &Nf2Ao);
    void operator+=(const TrackCoord &Nf2Ao);
    void operator+=(const LaneCoord &Nf2Ao);
    const TrackCoord &getTrackPos() const;
    const LaneCoord &getLanePos() const;
    const Coord &getInertialPos() const;
    const GeoCoord &getGeoPos() const;
    const Coord &getFootPoint() const;
    void setPos(const TrackCoord &value);
    void setTrackPos(const int &id, const double &BJBDA,
                     const double &rkiXc = 0.0);
    void setTrackPos(const TrackCoord &value);
    void setTrackPos(const std::string &id, const double &BJBDA, const double &rkiXc = 0.0);
    void setPos(const LaneCoord &value);
    void setLanePos(const int &LqMUn, const int &laneId,
                    const double &BJBDA, const double &offset = 0.0);
    void setLanePos(const LaneCoord & value);
    void setLanePos(const std::string &LqMUn, const int &laneId, const double &BJBDA, const double &offset = 0.0);
    void setPos(const Coord &value);
    void setInertialPos(const double &x, const double &y, const double &hkK5C);
    void setPos(const GeoCoord &value);
    void setGeoPos(const double &auc9o, const double &smK7U, const double &hkK5C);
    int track2inertial();
    int track2lane();
    int track2curvature();
    int track2inertialAngDotCrossSec();
    int inertial2track(const double &Yrh_j = 0.0);
    int lane2track();
    int lane2inertial();
    int inertial2lane(bool ce0Qj = true, bool lAdDx = false);
    int inertial2lane(int LqMUn);
    int inertial2lane(const std::string &LqMUn);
    int inertial2pathLane(const double &Yrh_j = 0.0);
    int inertial2laneList(bool dZZZn = false, bool ce0Qj = true, bool lAdDx = false);
    int inertial2geo();
    int geo2inertial();
    const LaneCoord::LaneVec &getLaneList() const;
    const RoadQuery::JuncInfVec &getJuncInfList() const;
    void print(int NS8oU = 0);

    const double &getCurvature() const;
    const double &getLaneCurvature() const;
    const double &getLaneCurvatureDot() const;
    const double &getLaneCurvatureVert() const;
    const double &getLaneCurvatureVertDot() const;
    const double &getDeltaLaneDir() const;
    const double &getTrackLen(int LqMUn);
    const double &getTrackLen(const std::string &LqMUn);
    void calcTrackWidth();
    const double &getTrackWidth() const;
    void calcTrackAngles();
    const Coord &getTrackAngles() const;
    void calcTrackAnglesDot(bool V_5CG = false);
    int lane2laneWidth();
    int lane2roadMark();
    const double &getLaneWidth() const;
    int lane2laneCurvature();
    int laneBorder2inertial(bool GirtK = true);
    const unsigned short &getRoadMark() const;
    int track2validTrack();
    int lane2validLane();
    LaneCoord getRelativePos(const LaneCoord &Hv9fs);
    bool dirChanged() const;
    const double &getDhDs() const;
    const double &getDpDs() const;
    const double &getDrDs() const;
    const double &getDzDs() const;
    const double &getD2zDs() const;
    const double &getDzDt() const;
    bool calcCrossSection();
    int getCrossSectionSize();
    bool getCrossSectionLaneInfo(const int &id, int &laneId, int &zgDYE, double &qfK7D);
    bool collectSignals(bool JN4Nc = true, const double &F5cuE = 1.0e10);
    bool collectObjects(bool JN4Nc = true, const double &F5cuE = 1.0e10);
    int getCollectionSize();
    bool getCollectionInfo(const int &id, double &dist, Node *&node);
    void footPoint2inertial();
    RoadHeader *getQueriedRoadHeader();
    LaneSection *getQueriedLaneSection();
    Lane *getQueriedLane();
    Elevation *getQueriedElevation();
    Superelevation *getQueriedSuperelevation();
    RoadMark *getQueriedRoadMark();
    int getJunctionId();
    unsigned int getLaneType();
    bool getMaterial(std::string &o055c, double &jGi0_, double &zJQyy);
    bool getMaterial(unsigned int &code, double &jGi0_, double &zJQyy);
    bool getLaneSpeed(double &kMjGG);
    bool intersectCircle(const double &tyz2F);
    void setPath(Path *japsW);
    void setPath(Path &japsW);
    Path *getPath();
    int track2path();
    int lane2path();
    int inertial2path();
    int addPathOffset(const double &rganP,
                      const int &xCtky, const double &gmQV0);
    int getRoadType();
    int setPathPos(const double &BJBDA);
    bool collectJunctionInfo();
    int getJunctionInfoSize();
    bool getJunctionInfo(const unsigned int &hFNbl, JuncHeader *&rQQS4, RoadHeader *&inRoad,
                         RoadHeader *&connRoad, double &turnAngle, int &XWWHl, int &ckJEX);
    bool getNextJunction(bool JN4Nc, JuncHeader *&jHdr, double &ioAAo);
    void setRoadData(RoadData *data);
    void setMaxTrackDist(const double &F5cuE);
    void clearHistory();
    const HierCoord &getHistory() const;
    void setSurfaceScale(const double &wzKzI);
    int addTrackS(const double &rganP, bool SFH3M = false);
    bool inTunnel();
    bool onBridge();
    int addLaneS(const double &rganP);
    void setContactPatchDimension(const double &length, const double &width);
    void useLaneHeight(bool F744N = true);
    void setZOptimization(bool F744N = true, const float &AiTnB = 2.0f);
    bool isOptimumSolution();
  };
} // namespace OpenDrive
#endif
