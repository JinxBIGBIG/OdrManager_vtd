
#ifndef _OPENDRIVE_ROAD_QUERY_HH
#define _OPENDRIVE_ROAD_QUERY_HH
#include <map>
#include "OdrRoadData.hh"
#include "OdrNode.hh"
#include "OdrRoadHeader.hh"
#include "OdrGeoNode.hh"
#include "OdrJuncHeader.hh"
#include "OdrLane.hh"
#include "OdrCoord.hh"
#include "OdrHierCoord.hh"
#include "OdrLaneCoord.hh"
#include "OdrGeoCoord.hh"
#include <vector>
namespace OpenDrive
{
  class SurfaceCRG;
  class RoadQuery
  {
  public:
    static const int
        RESULT_NOT_ON_ROAD = 0;
    static const int RESULT_ON_ROAD = 1;
    static const int
        RESULT_ERROR = 2;
    static const int RESULT_EXCEEDS_ROAD = 3;
    static const double
        maxTrackDist;
    static const double sMaxDeltaS;
    struct LaneInfoStruct
    {
      int id;
      int
          type;
      double width;
      Lane *lane;
    };
    typedef std::vector<struct LaneInfoStruct> LaneInfVec;
    struct ResultInfoStruct
    {
      Node *node;
      double dist;
    };
    typedef std::vector<
        struct ResultInfoStruct>
        ResultVec;
    struct JunctionInfoStruct
    {
      JuncHeader *jHdr;
      RoadHeader *inRoad;
      RoadHeader *connRoad;
      double turnAngle;
      int minLaneIn;
      int
          maxLaneIn;
    };
    typedef std::vector<struct JunctionInfoStruct> JuncInfVec;
  private:
    std ::string mName;
    RoadData *mRoadData;
    double mMaxTrackPos;
    double mCurvature;
    double mLaneCurvature;
    double mLaneCurvatureDot;
    double mLaneCurvatureVert;
    double mLaneCurvatureVertDot;
    double mLaneDeltaHdg;
    double mLaneWidth;
    double mLaneHeight;
    typedef std::map<unsigned int, Node *> NodeMap;
    NodeMap mNodeAccessMap;
    typedef std::pair<NodeMap::iterator, bool> NodeMapErr;
    HierCoord mHierPos;
    unsigned short
        mRoadMark;
    Coord mFootPoint;
    double mSTolerance;
    bool mTest;
    bool mExplicitRoadData;
    unsigned int mDebugMask;
    double mMaxTrackDist;
    double mSurfaceScale;
    bool
        mAllowLaneTypeRail;
    bool mAllowLaneTypeCar;
    double mContactPatchLength;
    double
        mContactPatchWidth;
    bool mUseContactPatch;
    bool mUseLaneHeight;
    bool
        mMinimizeDeltaZ;
    float mZTolerance;
    bool mOptimumSolution;
  public:
    explicit RoadQuery();
    explicit RoadQuery(const std::string &name);
    virtual ~RoadQuery();
    void
    setRootNode(Node *node);
    int track2inertial(Coord &Tlj7t, const TrackCoord &HQGFE);
    int track2inertialSimple(Coord &Tlj7t, const TrackCoord &HQGFE, RoadHeader *REKut);
    int inertial2track(const Coord &Tlj7t, TrackCoord &HQGFE, const double &Yrh_j = 0.0);
    int inertial2lane(const Coord &Tlj7t, LaneCoord &Hv9fs);
    int inertial2lane(const Coord &Tlj7t, LaneCoord::LaneVec &D7gpt, bool dZZZn = false);
    int inertial2lane(const Coord &Tlj7t, LaneCoord &Hv9fs, std::vector<RoadHeader *> &LOM4N, bool Wy_Hs = true);
    int
    inertial2geo(const Coord &Tlj7t, GeoCoord &V5tBk);
    int geo2inertial(const GeoCoord &
                         V5tBk,
                     Coord &Tlj7t);
    const double &getMaxTrackPos() const;
    double getCurvature();
    bool
    getLaneCurvature(double &j6pBH, double &Kv87d, double &It5y8, double &KGtll);
    double
    getDeltaLaneDir();
    double getTrackWidth(const TrackCoord &HQGFE);
    int
    getTrackHeading(const TrackCoord &HQGFE, double &jwOMF);
    int getTrackAngles(const TrackCoord &HQGFE, Coord &svZro);
    int getTrackAnglesDot(const TrackCoord &HQGFE, Coord
                                                       &svZro);
    int getPitchAndZ(const TrackCoord &HQGFE, double &s4E1B, double &hkK5C);
    int
    getPitch(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &s4E1B);
    int getPitchDot(
        RoadHeader *wR6_y, const TrackCoord &HQGFE, double &QkuuH, double &G8lrl);
    int
    getPitchAndZAndPitchDot(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &s4E1B,
                            double &hkK5C, double &QkuuH, double &G8lrl);
    int getRoll(const TrackCoord &HQGFE,
                double &hgSHv);
    int getRoll(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &hgSHv);
    int getRollDot(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &PwesK);
    int
    getRollAndRollDot(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &hgSHv, double &PwesK);
    int lane2track(LaneCoord &Hv9fs);
    int track2lane(LaneCoord &Hv9fs, RoadHeader
                                         *M8Gux = 0);
    double track2laneOffset(const LaneCoord &Hv9fs, double &xPO_v, RoadHeader *M8Gux = 0);
    int track2curvature(const TrackCoord &HQGFE);
    int lane2curvature(const LaneCoord &Hv9fs);
    const double &getLaneWidth() const;
    const double &getLaneHeight()
        const;
    int lane2laneWidth(const LaneCoord &Hv9fs);
    int lane2roadMark(const LaneCoord &Hv9fs);
    const unsigned short &getRoadMark() const;
    int track2validTrack(
        TrackCoord &HQGFE, bool &dirChanged, bool SFH3M = false);
    int track2validTrack(
        TrackCoord &HQGFE);
    int lane2validLane(LaneCoord &Hv9fs, bool &dirChanged);
    int
    lane2validLane(LaneCoord &Hv9fs);
    bool getCrossSection(const TrackCoord &HQGFE,
                         LaneInfVec &cxBiL);
    LaneSection *track2laneSection(const TrackCoord &HQGFE,
                                   RoadHeader *M8Gux = 0, bool checkSide = false, const int &laneId = 0);
    Lane *getLaneFromId(
        LaneSection *H4prs, int id);
    int track2inertialAngDotCrossSec(const TrackCoord &
                                         HQGFE,
                                     Coord &Tlj7t, Coord &oUumu, LaneInfVec &cxBiL);
    RoadHeader *trackId2Node(const int id);
    RoadHeader *trackId2Node(const unsigned int id);
    RoadHeader *trackId2Node(
        const std::string &id);
    RoadHeader *trackId2Node(const TrackCoord &HQGFE);
    bool
    getSignals(const TrackCoord &HQGFE, bool JN4Nc, const double &F5cuE, ResultVec &JWpF7);
    bool getObjects(const TrackCoord &HQGFE, bool JN4Nc, const double &F5cuE, ResultVec &JWpF7);
    const Coord &getFootPoint() const;
    Node *getQueriedNodeOfType(const unsigned int type);
    bool getMaterial(const LaneCoord &Hv9fs, unsigned int &code, double &jGi0_,
                     double &zJQyy, std::string &o055c);
    bool intersectCircle(const Coord &wiBPl, const double &tyz2F, LaneCoord::LaneVec &D7gpt);
    void setTolerance(const double &FfN9a);
    bool getLaneSpeed(const LaneCoord &Hv9fs, double &kMjGG);
    int getRoadType(const LaneCoord &Hv9fs);
    bool collectJuncInfo(JuncHeader *jHdr, JuncInfVec &JWpF7,
                         RoadHeader *inRoad = 0);
    bool getNextJunction(const TrackCoord &HQGFE, bool JN4Nc,
                         JuncHeader *&jHdr, double &ioAAo, RoadHeader *&inRoad);
    void setRoadData(RoadData *data);
    void setDebugMask(unsigned int GhT88);
    void setMaxTrackDist(const double &F5cuE);
    void clearHistory();
    void fillHistory(const HierCoord &SkShA);
    const HierCoord &
    getHistory() const;
    void setSurfaceScale(const double &wzKzI);
    int addTrackS(
        TrackCoord &HQGFE, const double &rganP, bool SFH3M = false);
    void setValidLaneTypes(
        bool ce0Qj = true, bool lAdDx = false);
    bool inTunnel(const LaneCoord &Hv9fs);
    bool
    onBridge(const LaneCoord &Hv9fs);
    int addLaneS(LaneCoord &Hv9fs, const double &rganP);
    void setContactPatchDimension(const double &length, const double &width);
    void
    useLaneHeight(bool F744N = true);
    void setZOptimization(bool F744N = true, const float
                                                 &AiTnB = 2.0f);
    bool isOptimumSolution() const;
  private:
    int st2pitchAndZ(RoadHeader *
                         wR6_y,
                     const double &BJBDA, const double &rkiXc, double &s4E1B, double &hkK5C, const double &hgSHv = 0.0);
    GeoHeader *track2geoHeader(RoadHeader *wR6_y, const TrackCoord &
                                                      HQGFE);
    GeoHeader *track2geoHeader(const TrackCoord &HQGFE);
    Elevation *s2elevation(
        RoadHeader *wR6_y, const double &BJBDA);
    LateralShape *EBWwH(RoadHeader *wR6_y, const double &BJBDA, const double &rkiXc = 0);
    SurfaceCRG *s2surface(RoadHeader *wR6_y, const double &BJBDA, const unsigned short &TCz3u = 0xffff);
    Superelevation *s2superelevation(
        RoadHeader *wR6_y, const double &BJBDA);
    Crossfall *track2crossfall(RoadHeader *wR6_y,
                               const TrackCoord &HQGFE);
    Crossfall *track2crossfall(const TrackCoord &HQGFE);
    int
    track2InertialPitchAndZ(RoadHeader *wR6_y, Coord &Tlj7t, const TrackCoord &HQGFE);
    int
    trackAndRoll2PitchAndZ(RoadHeader *wR6_y, const TrackCoord &HQGFE, const double &hgSHv, double &s4E1B, double &hkK5C);
    double getLaneWidth(Lane *lane, const double &
                                        rganP);
    double getLaneWidth(Lane *lane, const double &rganP, double &lseyi, double &L9KFn);
    double getLaneWidthAndRoadMark(Lane *lane, const double &rganP);
    unsigned short getRoadMark(Lane *lane, const double &rganP);
    bool checkForRoadData();
    int
    getLaneOnPreviousRoad(const LaneCoord &Hv9fs, RoadHeader *wR6_y = 0);
    int
    getLaneOnPreviousRoad(const LaneCoord &Hv9fs, RoadHeader *wR6_y, bool &OjzDY);
    int
    getLaneOnNextRoad(const LaneCoord &Hv9fs, RoadHeader *wR6_y = 0);
    int
    getLaneOnNextRoad(const LaneCoord &Hv9fs, RoadHeader *wR6_y, bool &OjzDY);
    int
    inertial2lane(RoadHeader *wR6_y, GeoHeader *xjOaT, const Coord &Tlj7t, LaneCoord &Hv9fs, bool oWK6t = true);
    Lane *laneId2Node(int laneId);
    const double &s2tolS(const double &
                             BJBDA,
                         RoadHeader *REKut);
    bool laneSpeedFromRoadType(const LaneCoord &Hv9fs, double &
                                                           kMjGG);
    double lane2laneHeight(const LaneCoord &Hv9fs);
    bool Ekmkk(RoadHeader *REKut, const TrackCoord &HQGFE, double &hkK5C);
    double geoNode2curvature(GeoNode *pWj2K,
                             RoadHeader *REKut, const double &BJBDA);
  };
} // namespace OpenDrive
#endif
