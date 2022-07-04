#include <vector>
#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "OdrGeoCoord.hh"
#include "OdrManagerLite.hh"
#include "BaseNodes/OdrNode.hh"
#include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrObject.hh"
#include "BaseNodes/OdrJuncHeader.hh"
#include "BaseNodes/OdrBridge.hh"
#include "BaseNodes/OdrElevation.hh"
#include "BaseNodes/OdrLane.hh"
#include "BaseNodes/OdrLaneSection.hh"
#include "BaseNodes/OdrRoadLink.hh"
#include "BaseNodes/OdrJuncLink.hh"
#include "BaseNodes/OdrJuncLaneLink.hh"
#include "BaseNodes/OdrSignal.hh"
#include "BaseNodes/OdrRoadMark.hh"
#include "OdrTrackCoord.hh"
#include "OdrLaneCoord.hh"


// the lane info at the position relativeS in front of current vehicle


struct posInfo {
  int laneId;
  int roadId;
  int junctionId;
  int roadlength;
  double sectionS;
  unsigned int laneType;
  double laneSpeed;
  double trackS;
  double trackT;
  double roadheading;
  double ElvmS;
  double ElvmSEnd;
  double ElvmA;
  double ElvmB;
  double ElvmC;
  double ElvmD;
  double laneWidth;
  OpenDrive::Lane* lane;
  OpenDrive::RoadHeader* road;
  OpenDrive::LaneSection* section;
  OpenDrive::EnRoadMarkType roadmarkType;   
  OpenDrive::EnRoadMarkColor roadmarkColor;
  OpenDrive::Elevation* posElevation;
};

struct Point
{
  double x{0.};
  double y{0.};
  double z{0.};

  Point() = default;

  Point(double _x, double _y, double _z)
    : x(_x)
    , y(_y)
    , z(_z)
  {}
};

class getPosInfo {
public:
  getPosInfo();
  bool init(const std::string& odrFile);

  bool getInertialPosInfo(Point p);

  // return 0 means not in same lane, otherwise in samelane
  // check if two points are in the same lane
  // considering two points in different laneSection and different road
  // considering points in junction, road overlap
  // vector<struct> Preprocess1(Point p1,float heading1);
  int** GetJuncDir(Point p1,float heading1);
  int **Preprocess(Point p1,float heading1);
  int** JuncLinkRoad(Point p1,float heading1);
  

  int isInBackLane(Point p1, Point p2 ,float heading1,float *length) ;

  int isInFrontLane(Point p1, Point p2 ,float heading1,float *length) ;

  //  int drivingDirRelToRoad(bool isRHT);

  // int isInSameLane(Point p1, Point p2, bool isInFront);

  bool isBackCar(Point p1, Point p2,float heading1,float heading2,float *length) ;

  bool isFrontCar(Point p1, Point p2,float heading1,float heading2,float *length) ;

  bool isOnRoadMark(const Point& p);

  bool isSameDirectionWithRoad(Point p,float heading1);

  bool isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType);

  bool isInSpecialRoadType(Point p, OpenDrive::EnRoadType roadType);

  bool isInSce2_45_6(Point p);

  bool isInSce2_45_7(Point p);

  bool isInSce2_45_8(Point p);

  bool isInSce2_45_9(Point p);

  bool isInSce2_46_8(Point p,float heading1);

  bool isInSce2_59_2(Point p);

  bool isInSce2_46_7(Point p);

  bool isInSec1_47_1(Point p,float heading1,float *SideWalkDis);

  bool isInSec2_46_14(Point p);

  bool isInSec2_46_4(Point p);

  bool isInSec2_46_5(Point p);

  bool isInSce2_46_3(Point p);

  bool isInSce2_81_x(Point p);



  posInfo getInfo() const;

  // get the driving direction relative to road referrence coordinate s-t
  // isRHT : is Right Hand Traffic
  // return 1  means increasing
  //        -1 means decreasing
  // int drivingDirRelToRoad(bool isRHT = true);

private:
  OpenDrive::OdrManager m_manager;
  posInfo m_posInfo;
};

