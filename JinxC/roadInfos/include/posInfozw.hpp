#include <vector>
#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "OdrGeoCoord.hh"
#include "OdrManagerLite.hh"
#include "BaseNodes/OdrNode.hh"
#include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrJuncHeader.hh"
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
// 不换道的情况下，车辆前方一段距离的车道信息，有点类似于胡师兄的 routlist
// 每运动一步就更新，工作量是不是太大，大量重复工作？可不可以增量更新？？

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
  OpenDrive::Lane* lane;
  OpenDrive::RoadHeader* road;
  OpenDrive::LaneSection* section;
  OpenDrive::EnRoadMarkType roadmarkType;   
  OpenDrive::EnRoadMarkColor roadmarkColor;
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
  

  int isInBackLane(Point p1, Point p2 ,float heading1,float *length) ;

  int isInFrontLane(Point p1, Point p2 ,float heading1,float *length) ;

  //  int drivingDirRelToRoad(bool isRHT);

  // int isInSameLane(Point p1, Point p2, bool isInFront);

  bool isBackCar(Point p1, Point p2,float heading1,float heading2,float *length) ;

  bool isFrontCar(Point p1, Point p2,float heading1,float heading2,float *length) ;


  bool isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType);

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

