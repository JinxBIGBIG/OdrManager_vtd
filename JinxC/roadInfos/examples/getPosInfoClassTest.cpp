#include "OpenDRIVE.hh"
#include "BaseNodes/OdrLaneSection.hh"
#include "BaseNodes/OdrElevation.hh"


// #include "posInfo.hpp"
#include "posInfo.hpp"
#include <iostream>
#include <cmath>
using namespace OpenDrive;
using namespace std;
// #include <sys/time.h> //timeval

int main(int argc, char** argv) {
  // OpenDrive::OdrManager manager;
  // // bool flag = true;
  // std::string odrFile = "D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr";
  // getPosInfo posManager;
  // bool ret = posManager.init(odrFile);

  // OpenDrive::Position* pos = manager.createPosition();
  // manager.activatePosition(pos);
  //common
  OpenDrive::OdrManager manager;
  bool flag = manager.loadFile("D:\\gx\\evaluate\\roadInfos\\data\\MT_intersection_ego_1lane.xodr");
  if (flag)
    cout << "load file successful." << endl;
  else
    cout << "load file wrong." << endl;

  OpenDrive::Position* pos = manager.createPosition();
  manager.activatePosition(pos);

  getPosInfo posManager;
  // //function3_7_5example
  // float pi=4*atan(1);
  // Point p1(5948.24,-3000.27,0);
  // Point p2(5909.09,-2967.52,0);
  // float heading1=3*(pi/4);
  // float heading2=3*(pi/4);
  // float * length= new float;
  // float  visibility=40;
  // int snow=0;
  // int rainy=0;
  // int hail=0;
  // bool function3_7_5flag=posManager.function3_7_5(p1, p2, heading1,heading2,length, visibility,snow, rainy,hail);
  // bool isInSce2_46_7flag=posManager.isInSce2_46_7(p1);
  // std::cout << "isInSce2_46_7flag" << isInSce2_46_7flag<<std::endl;
  // float p2x=2834.63;
  // float p2y=-7114.21;
  // // float p2z=0;
  // std::cout << "m_posInfo" << std::endl;

  // double p1x=115.979;
  // double p1y=5.51219;
  // double p1z=0;
  // Point p1(p1x,p1y,p1z);
  // bool test=posManager.test(p1);
  // bool testflag=posManager.findLimitedSpeed();


  // manager.setInertialPos(3885.53,-2964.57,-4.78384);
 
  // OpenDrive::LaneSection* sec = posManager.section;
  // OpenDrive::LaneSection* secNext = (OpenDrive::LaneSection*)sec->getRight();

  // if (!ret) {
  //   std::cout << "load xodr file error!" << std::endl;
  //   return -1;
  // }

  // timeval start, end;
  // gettimeofday(&start, NULL);


  // posManager.getInertialPosInfo(p1);

  // 下陡坡
  // OpenDrive::OdrManager Manager;
  // OpenDrive::Position* pos1 = Manager.createPosition();
  // Manager.activatePosition(pos1);

  // manager.setLanePos(1,1,402);
  // bool result = manager.lane2inertial();
  // // // if (!result)
  // // // {
  // // //   std::cout << "-1" <<std::endl;
  // // // }
  // double p1x=manager.getInertialPos().getX();
  // double p1y=manager.getInertialPos().getY();
  // double p1z=manager.getInertialPos().getZ();
  // Point p1(p1x, p1y, p1z);
    Point p1(114.798, 5.182, 0);
  // float heading=0.9;
  float *StartHeading =new float ;
  float *EndHeading =new float ;
  // float * MySlope =new float ;
  // float * MySlopeHeight =new float ;
  // std::cout << "p:" <<"("<< p1x<<","<<p1y<<","<<p1z<<")"<<std::endl;
  bool isInSce2_46_7flag=posManager.isInSce2_46_7(p1,StartHeading,EndHeading);
  std::cout << "isInSce2_46_7flag:" << isInSce2_46_7flag<<std::endl;
  // std::cout << "MySlope:" << *MySlope<<std::endl;
  std::cout << "StartHeading:" << *StartHeading<<std::endl;
  std::cout << "EndHeading:" << *EndHeading<<std::endl;
  //   bool isInSec2_46_5=posManager.isInSec2_46_5(p1);
  //   std::cout << "isInSec2_46_5: " << isInSec2_46_5<< std::endl;

    // double x=p1x;
    // double y=p1y;
    // double x0=  
    // double s=y*cos(xita)-x*sin(xita)-y0;
    // double t=x*cos(xita)+y*sin(xita)-x0;





    // float speed=98;
    // float * length= new float;
    // int time =1;
    // bool juncflag=posManager.function3_7_4(p1,p2,pi/4,pi/4,length,time);
    // std::cout << "function3_7_4: " << juncflag << std::endl;
    // std::cout << "length: " << (*length )<< std::endl;
  

  // std::cout << "m_posInfo.RoadID:" << Manager.getRoadHeader()->mId<< std::endl;
  // std::cout << "m_posInfo.SectionS:" <<Manager.getLaneSection()->mS<< std::endl;
  // std::cout << "m_posInfo.laneId:" <<Manager.getLane()->mId<< std::endl;


  // OpenDrive::TrackCoord posTrack = manager.getTrackPos();
  // double posTrackS = posTrack.getS();
  // double posTrackT = posTrack.getT();
  // double posTrackH = posTrack.getH();
  // std::cout << "current position track s: " << posTrackS << std::endl;
  // std::cout << "current position track t: " << posTrackT << std::endl;
  // std::cout << "current position track H: " << posTrackH << std::endl;


  // Point p1(2834.63,-7114.21,0);
  
  // Point p2(12.5961430875964,0.0268242243256758,-0.002803588);
  // std::cout << "road id : " <<  << std::endl;

  // Point p1(128.001,-4.584,0);
  // Point p2(5909.09,-2967.52,0);
  // float heading1=3*(pi/4);
  // float heading2=3*(pi/4);
  // // float heading1 = 45;
  // // float heading2 = 45;
  
  // float * length= new float;
  // bool  isfrontcarFlag = posManager.isFrontCar(p1,p2,heading1,heading2,length);
  // std::cout << "is front car " << isfrontcarFlag << std::endl;


  // bool isSameDirectionWithRoad= posManager.isSameDirectionWithRoad(p1,(pi/4));

  // bool isSameDirectionWithRoad= posManager.isInSec2_46_5(p1);
  
  // double p1x=338.4797781;
  // double p1y=0.757743361;
  // double p1z=0;
  // Point p1(p1x,p1y,p1z);


  // float heading1=pi/4;
  // float * SideWalkDis= new float;
  // bool isSameDirectionWithRoad= posManager.isInSec1_47_1( p1,  heading1, SideWalkDis);
  
  
  // std::cout << "SideWalkDis " << * SideWalkDis << std::endl;
  //  std::cout << "if SideWalkDis : " << isSameDirectionWithRoad << std::endl;


  // int flag = posManager.isInSameLane(p1, p2);
  // if (flag > 0) 
  //   std::cout << "the two points are in the same lane." << std::endl;
  // else
  //   std::cout << "the two points are not in the same lane." << std::endl;
  
  // std::cout << "flag is: " << flag << std::endl;

  // OpenDrive::OdrManager Manager;
  // OpenDrive::Position* pos1 = Manager.createPosition();
  // Manager.activatePosition(pos1);
  // Manager.setLanePos(21, -2, 0.1);
  // bool result = Manager.lane2inertial();
  // double p1x=Manager.getInertialPos().getX();
  // double p1y=Manager.getInertialPos().getY();
  // double p1z=Manager.getInertialPos().getZ();
  // Point p2(p1x, p1y, p1z);

  // Point p1(3926.63,-2943.46, 0);
  // float pi=4*atan(1);
  // // std::cout << "isDrivingDown start." << std::endl;
  // bool isDrivingDown=posManager.isDrivingDownSteep(p1 ,3*(pi/4));
  // if (isDrivingDown)
  //   std::cout << "this car is DrivingDown." <<isDrivingDown<< std::endl;
  // else
  //   std::cout << "this car is not DrivingDown." << std::endl;

  // std::cout << "isTopRam start." << std::endl;
  // bool isTopRam=posManager.isTopRamp(p1);
  // if (isTopRam)
  //   std::cout << "this point is at TopRamp." << std::endl;
  // else
  //   std::cout << "this point is not at TopRamp." << std::endl;

  // std::cout << "isTurning start." << std::endl;
  // bool isTurn=posManager.isTurning(p1 );
  // if (isTurn)
  //   std::cout << "this car is Turning." << std::endl;
  // else
  //   std::cout << "this car is not Turning." << std::endl;

  // bool resultwh  = posManager.isInSce2_45_6(p2);
  // std::cout <<resultwh << std::endl;
  
  // bool isInBusLane = posManager.isInSpecialLaneType(p1, OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS , OpenDrive::EnAccessRestriction::ODR_LANE_ACCESS_RESTRICT_BUS);
  
  // if (isInBusLane)
  //   std::cout << "this point is in Bus lane." << std::endl;
  // else
  //   std::cout << "this point is not in Bus lane." << std::endl;

  // bool isOnRoadmark = posManager.isOnRoadMark(p1);
  // if (isOnRoadmark)
  //   std::cout << "this point is on roadmark." << std::endl;
  // else
  //   std::cout << "this point is not on roadmark." << std::endl;

  // gettimeofday(&end, NULL);
  // double elasped = end.tv_sec - start.tv_sec + (end.tv_usec - start.tv_usec) / 1000000.0;     // unit  sec
  // std::cout << "the time used for parsing is: " << elasped << " s" << std::endl;

  return 0;
}


