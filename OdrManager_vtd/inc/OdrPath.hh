
#ifndef _OPENDRIVE_PATH_HH
#define _OPENDRIVE_PATH_HH
#include <string>
#include <vector>
#include <OdrCoord.hh>
#include <OdrLaneCoord.hh>
namespace OpenDrive
{
  class Lane;
  class RoadQuery;
  class RoadHeader;
  class Path
  {
    static const int sMaxSearchDepth = 100;
  public:
    class Leg
    {
    public:
      class LaneList
      {
      public:
        struct LaneInfo
        {
          Lane *mLane;
          Lane *mSuccessorLane;
          bool mHasSuccessor;
          double mRemainingLength;
          LaneList *mParent;
          struct LaneInfo *mSuccLaneInfo;
          struct LaneInfo *mPredLaneInfo;
          unsigned int mIndex;
        };
        typedef std::vector<struct LaneInfo *> LaneVector;
        typedef LaneVector::iterator LaneVectorIterator;
        LaneVector mLaneVec;
        double mSRel;
      public:
        explicit LaneList();
        virtual ~LaneList();
        void addLane(Lane *lane, bool sLORZ);
        void setSRel(const double &BJBDA);
        const double &getSRel() const;
        void print();
        unsigned int size();
        void clear();
        LaneInfo *getInfoAtIndex(unsigned int hFNbl);
        LaneInfo *getInfoForLaneId(int laneId);
        LaneInfo *getNeighborInfo(
            LaneInfo *MA_pl, int xCtky);
        void setSuccessorAtIndex(Lane *XPRvc, unsigned int hFNbl);
        void setRemainingLengthAtIndex(const double &len, unsigned int hFNbl);
        void
        setSuccessorLaneList(LaneList *XPRvc);
      };
    public:
      RoadHeader *mRoadHdr;
      bool mForward;
      double mRoadS;
      double mPathS;
      double mLen;
      typedef std::vector<LaneList *> LaneListVector;
      typedef LaneListVector::iterator LaneListIterator;
      LaneListVector mLaneListVec;
    public:
      explicit Leg(RoadHeader *wR6_y, bool JN4Nc, const double &BJBDA,
                   const double &len);
      explicit Leg(bool vYR0i);
      virtual ~Leg();
      void revert();
      void print();
      void calcLaneSequence(const double &byoWw, bool kOKsv);
      void clearLaneLists();
      unsigned int getSizeLaneList();
      LaneList *getLaneListAtIndex(unsigned int hFNbl);
      LaneList *s2laneList(const double &BJBDA);
    };
  private:
    std::string mName;
    typedef std::vector<Leg *> LegVector;
    typedef LegVector::iterator LegVectorIterator;
    LegVector mLegVec;
    LegVectorIterator mLegIt;
    bool mValid;
    double mTotalLen;
    double mCurrentS;
    double mStartS;
    bool mOnPath;
    RoadQuery *mRoadQuery;
    LaneCoord mLanePos;
    Coord mInertialPos;
    Leg *mCurrentLeg;
    bool mOnPathOld;
    double mSOld;
    LaneCoord mLanePosOld;
    Coord mInertialPosOld;
    Leg *mLegOld;
    bool mWarnPathPos;
  public:
    explicit Path(const std::string &name);
    virtual ~Path();
    const std::string &getName() const;
    Path *getCopy();
    void reset();
    void loop();
    bool addWaypoint(const TrackCoord &KauWq,
                     bool XfG8J = false);
    int getTrack() const;
    const TrackCoord &getTrackPos() const;
    const LaneCoord &getLanePos() const;
    double getLegStartS() const;
    double getS() const;
    bool getFwd() const;
    TrackCoord getTrackPosAtS(const double &BJBDA);
    bool getFwdAtS(const double &BJBDA);
    bool addS(const double &BJBDA);
    bool addOffsetPos(const double &
                          rganP,
                      const int &xCtky, const double &qfVP6);
    void setStartS(const double &BJBDA);
    bool setPos(const double &BJBDA);
    double getMaxCurvatureInRange(const double &start, const double &_aUXU);
    void print();
    bool posValid() const;
    bool
    calcFromTrackPositions(const TrackCoord &q_y7v, const TrackCoord &KauWq, bool XfG8J = false, bool JN4Nc = true);
    void invert();
    const double &getLength() const;
    bool sameTrackDir(const double &iL5Gj, const double &ikhot);
    bool addPath(Path *japsW);
    bool addPath(Path &japsW);
    bool pos2s(const Coord &wiBPl);
    bool pos2s(const LaneCoord &Hv9fs, const double &Yrh_j = 0.0);
    bool nextLeg();
    void getRoads(std::vector<RoadHeader *> &y97wI, const double &Yrh_j = 0.0);
    void pushPos();
    void popPos();
    bool addLeg(Leg *UuV75);
    Leg *getCurrentLeg() const;
    Path::Leg::LaneList::LaneInfo *
    pos2LaneInfo();
  private:
    Path(const Path &sdNGn);
    Path &operator=(const Path &sdNGn);
    void clearLegVec(std::vector<Leg *> &Fgv7w);
    void cleanCopyLegVec(LegVector &szJ7I,
                         LegVector &SkShA, LegVector &BLJaz);
    bool addLeg(unsigned int LqMUn, bool JN4Nc, const double &BJBDA);
    bool addLeg(RoadHeader *wR6_y, bool JN4Nc, const double &BJBDA);
    bool addLeg(unsigned int LqMUn, bool JN4Nc);
    bool addLeg(RoadHeader *wR6_y, bool JN4Nc);
    bool addLeg(unsigned int LqMUn);
    bool addLeg(RoadHeader *wR6_y);
    bool
    checkConnectionsForTargetPos(RoadHeader *REKut, const TrackCoord &KauWq, bool &forward, RoadHeader *II6ml = NULL, int depth = 0, int ls_cG = 0, bool ChBDm = false);
    void
    invertOrder();
    Leg *s2Leg(const double &BJBDA);
    Leg *pos2Leg(const LaneCoord &Hv9fs,
                 const double &Yrh_j = 0.0, bool TE61x = false);
    void optimize();
    void calcLaneLists();
    void updateLaneValidity();
    void calcLaneLengths();
  };
} // namespace OpenDrive
#endif
