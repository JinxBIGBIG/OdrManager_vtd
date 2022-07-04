#include "OdrManager.hh"

#include <iostream>
using namespace std;


unsigned int getLaneInfoFromInertialPos(OpenDrive::OdrManager& manager, 
                                        double& x, double& y, double& z) {
  OpenDrive::Position* pos = manager.createPosition();
  manager.activatePosition(pos);

  manager.setInertialPos(x, y, z);
  bool result = manager.inertial2lane();
  unsigned int laneType;
  if (result) {
    laneType = manager.getLaneType();
  }
  x = 10;
  return laneType;
}


int main(int argc, char** argv) {
  OpenDrive::OdrManager manager;
  manager.loadFile("D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr");
  double x = 6045;
  double y = -2011;
  double z = 0;
  unsigned int lane_type = getLaneInfoFromInertialPos(manager, x, y, z);
  

  cout << "laneType is: " << lane_type << endl;

  return 0;
}


