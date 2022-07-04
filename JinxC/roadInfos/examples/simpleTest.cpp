#include "OdrManager.hh"
#include "BaseNodes/OdrNode.hh"
#include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrElevation.hh"
#include "BaseNodes/OdrLane.hh"
#include "BaseNodes/OdrRoadLink.hh"
#include "BaseNodes/OdrRoadMark.hh"
#include "OdrTrackCoord.hh"
#include "OdrLaneCoord.hh"

#include <iostream>
#include <cmath>
using namespace std;

int main(int argc, char** argv) {
  cout << "simple test" << endl;
  OpenDrive::OdrManager manager;
  // bool flag = true;
  bool flag = manager.loadFile("D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr");
  if (flag)
    cout << "load file successful." << endl;
  else
    cout << "load file wrong." << endl;

  OpenDrive::Position* pos = manager.createPosition();
  manager.activatePosition(pos);

  float p2x=2834.63;
  float p2y=-7114.21;
  float p2z=0;

  manager.setInertialPos(2834.63,-7114.21,0);
  // manager.setInertialPos(3885.53,-2964.57,-4.78384);
  bool result = manager.inertial2lane();
  if (result)
  {
    std::cout << "111 " << std:: endl;
    OpenDrive::TrackCoord posTrack = manager.getTrackPos();
    double posTrackS = posTrack.getS();
    double posTrackT = posTrack.getT();
    double posTrackH = posTrack.getH();
    std::cout << "current position track s: " << posTrackS <<std:: endl;
    std::cout << "current position track t: " << posTrackT << std::endl;
    std::cout << "current position track H: " << posTrackH << std::endl;
    std::cout << "111 end" << std:: endl;
  }
  
  

  manager.print();

  // // manager.attach();
  

  if (result) {
    // OpenDrive::RoadHeader* header = reinterpret_cast< OpenDrive::RoadHeader* >(manager.getRoadHeader());
    OpenDrive::RoadHeader* header = manager.getRoadHeader();
    
    
    cout << "header->printData() : " << endl;
    header->printData();
    cout << endl;
  
    cout << "track id as string " << header->mIdAsString << endl;
    cout << "track id " << header->mId << endl;
    cout << "track lengh " << header->mLength << endl;
    cout << "junction no " << header->mJuncNo << endl;

    cout << "get track length: " << header->getLength() << endl;
    cout << "get track id: " << header->getId() << endl;
    cout << "get junction no: " << header->getJunctionNo() << endl;
    cout << "get track idstr: " << header->getIdStr() << endl << endl;

    OpenDrive::RoadLink* link = header->getSuccessorLink();
    OpenDrive::RoadHeader* ROAD = header->getSuccessor();
    cout << "print link info \n";
    
    OpenDrive::Lane* lane = manager.getLane();

    cout << "lane->printData() : " << endl;
    lane->printData();
    cout << endl;

    cout << "lane id is: " << lane->mId << endl;
    cout << "lane type is: " << lane->mType << endl;
    
    cout << "get lane id is: " << lane->getId() << endl;
    cout << "get lane type is: " << lane->getType() << endl;

    OpenDrive::TrackCoord posTrack = manager.getTrackPos();
    double posTrackS = posTrack.getS();
    double posTrackT = posTrack.getT();
    double posTrackH = posTrack.getH();
    cout << "current position track s: " << posTrackS << endl;
    cout << "current position track t: " << posTrackT << endl;
    cout << "current position track H: " << posTrackH << endl;

    OpenDrive::LaneCoord posLane = manager.getLanePos();
    cout << "current lane id: " << posLane.getLaneId() << endl
         << "current lane S: " << posLane.getS() << endl
         << "current lane T: " << posLane.getT() << endl
         << "offset from lane center: " << posLane.getOffset() << endl;
 
    double width = manager.getLaneWidth();
    cout << "current lane width is: " << width << endl;

    unsigned short roadmark = manager.getRoadMark();
    cout << "roadmark is: " << roadmark << endl;

    double dist = width / 2 - fabs(posLane.getOffset());
    const double MARKWIDTH = 0.12;
    cout << "dist is: " << dist << endl;
    if (dist < MARKWIDTH / 2) 
      cout << "  the point is on road mark." << endl;

    double speed = manager.getLaneSpeed() * 3.6;   // change to km/h
    cout << "lane speed is: " << speed << endl;

    OpenDrive::RoadMark* mark = manager.getQueriedRoadMark();
    cout << "raodmark color: " << mark->getColor() << endl;
    cout << "roadmark type: " << mark->mType << "  " << mark->getType() << endl;
    cout << "roadmark width: " << mark->mWidth << "  " << mark->getWidth() << endl;

    OpenDrive::Elevation* myElevation = manager.getElevation();
    cout << "myElevation ms: " << myElevation->mS << endl;
    if ((OpenDrive::Elevation* )myElevation->getLeft())
    {
      cout << "buweikong: " <<endl;
    }
    else
    {
      cout << "kong: " <<endl;
    }

    // OpenDrive::Elevation* myLastElevation = (OpenDrive::Elevation* )myElevation->getLeft();
    // cout << "myLastElevation ms: " << myLastElevation->mS << endl;
    // double elvLength=(myElevation->mSEnd)-(myElevation->mS);
    // cout << "melvLength: " << elvLength<< endl;
    // manager.getLane()->mId=manager.getLane()-> getSuccessor()->mId;  
    // // manager.getLane()->JuncLaneLink();
    // cout << "getLane: " << manager.getLane()->mId << endl;  
    // // cout << "getLane: " << manager.getLane()->JuncLaneLink()<< endl;  

    OpenDrive::GeoCoord myCoord(p2x, p2y, p2z);
    manager.setPos( myCoord );
    manager.lane2inertial();
    float roadheading=myCoord.getH();
    // cout << "roadheading: " << roadheading<< endl;
    manager.setLanePos();

  }

  return 0;
}


