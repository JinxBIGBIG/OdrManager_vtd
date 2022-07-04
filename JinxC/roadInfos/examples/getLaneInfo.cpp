#include "OdrManager.hh"
#include "BaseNodes/OdrNode.hh"
#include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrLane.hh"
#include "BaseNodes/OdrLaneSection.hh"

#include "posInfo.hpp"

#include <iostream>
using namespace std;


void getLaneInfoFromInertialPos(OpenDrive::OdrManager& manager, 
                                double x, double y, double z, 
                                posInfo& posinfo) {
  OpenDrive::Position* pos = manager.createPosition();
  manager.activatePosition(pos);

  manager.setInertialPos(x, y, z);
  bool result = manager.inertial2lane();

  if (result) {
    OpenDrive::RoadHeader* header = manager.getRoadHeader();
    if (!header) return;
    posinfo.roadId = header->getId();

    OpenDrive::Lane* lane = manager.getLane();
    if (!lane) return;

    double speed = manager.getLaneSpeed();
    posinfo.laneId = lane->getId();
    posinfo.laneSpeed = speed;
    posinfo.laneType = lane->getType();

    OpenDrive::LaneSection* section = manager.getLaneSection();
    if (!section) return;
    posinfo.sectionS = section->mS;
  }
}


int main(int argc, char** argv) {
  OpenDrive::OdrManager manager;
  manager.loadFile("..\\data\\Germany_2018.xodr");

  posInfo posinfo;
  getLaneInfoFromInertialPos(manager, 5948, -3000, 0, posinfo);

  cout << "posinfo roadId: " << posinfo.roadId << endl
       << "posinfo laneId: " << posinfo.laneId << endl
       << "posinfo laneSpeed: " << posinfo.laneSpeed << endl;

  return 0;
}






