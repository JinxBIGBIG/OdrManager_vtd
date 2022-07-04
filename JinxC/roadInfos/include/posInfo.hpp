#include <vector>
#include "tinyxml2.h"
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
#include "BaseNodes/OdrLaneAccess.hh"
#include "BaseNodes/OdrLaneSection.hh"
#include "BaseNodes/OdrRoadLink.hh"
#include "BaseNodes/OdrJuncLink.hh"
#include "BaseNodes/OdrJuncLaneLink.hh"
#include "BaseNodes/OdrSignal.hh"
#include "BaseNodes/OdrRoadMark.hh"
#include "BaseNodes/OdrRoadType.hh"
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
  
  bool isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType,OpenDrive::EnAccessRestriction laneAccessRestrictType);

  double findLimitedSpeed(Point p);

  bool isInSce2_46_8(Point p,float heading1, float *MySlope,float *MySlopeHeight);

  bool isInSce2_59_2(Point p);

  bool isInSce2_46_7(Point p,float *StartHeading,float *EndHeading);

  bool isInSec1_47_1(Point p,float heading1,float *SideWalkDis);

  bool isInSec2_46_14(Point p);

  bool isInSec2_46_4(Point p,float *MyRoadWidth);

  bool isInSec2_46_5(Point p,float *MyBridgeWidth);

  bool isInSec2_46_2(Point p);

  bool isInSec2_46_6(Point p1,Point p2,float heading1, float heading2,int* RoadID);

  bool isInSec1_46_2(Point p);

  int drivingDirRelToRoad(bool isRHT = true);

  int drivingDirRelToRoad(double lastS, double curS);

  std::vector<OpenDrive::Signal*> getAllSpeedSignsOfRoad(OpenDrive::RoadHeader* road);
  std::vector<OpenDrive::Signal*> getLiftSpeedSignsOfRoad(OpenDrive::RoadHeader* road);

  bool hasSpeedSignLimit(Point p);

  bool isInSce2_45_6(Point p);

  bool isInSce2_45_7(Point p);

  bool isInSce2_45_8(Point p);

  bool isInSce2_45_9(Point p);

  bool isInSce2_46_3(Point p);

  bool isInSpecialRoadType(Point p, OpenDrive::EnRoadType roadType);

  bool isInSce2_81_x(Point p);

  bool isInSce1_42_1(Point p);

  bool isInSce2_45_5(Point p);
  
  bool isInSec2_46_1(Point p);

  int isInSec2_47_1(Point pa11,Point pa12,float heading1); 

  bool isInSec1_51_1(Point p,float vehicleSpeed,float heading);

  bool isInSec3_6_6(Point p,float vehicleSpeed,int vehicleType,Point p2,float heading1);

  bool function2_80_1(Point  p1,Point  p2,float  heading1,float  heading2,float* length,float speed);

  bool function2_80_2(Point  p1,Point  p2,float  heading1,float  heading2,float* length,float speed);

  bool function3_7_1(Point  p1,Point  p2,float  heading1,float  heading2,float* length);

  bool function3_7_4(Point  p1,Point  p2,float  heading1,float  heading2,float* length,float time);

  bool isInSec1_47_2(Point p);

  int isInSec2_57_5(int laneId1,int laneId2,float heading1,double roadHeading); 

  bool function3_7_5(Point  p1,Point  p2,float  heading1,float  heading2,float* length,float  visibility,int snow,int rainy,int hail);

  bool function1_43_1(Point p1, Point p2,float heading1,float heading2,float *length) ;
  
  // bool isSameDirectionWithRoad(Point p,float heading1);

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

