#include "OdrManager.hh"

#include <iostream>
using namespace std;

// unsigned int getLaneInfoFromInertialPos(OpenDrive::OdrManager& manager, 
//                                 double x, double y, double z) {
//   OpenDrive::Position* pos = manager.createPosition();
//   manager.activatePosition(pos);

//   manager.setInertialPos(x, y, z);
//   bool result = manager.inertial2lane();
//   unsigned int laneType;
//   if (result) {
//     laneType = manager.getLaneType();
//   }
//   return laneType;
// }


int main(int argc, char** argv) {
  cout << "simple test." << endl;
  OpenDrive::OdrManager* manager;
  manager->loadFile("../data/Germany_2018.xodr");
  cout << "simple test." << endl;
  // manager.loadFile("../data/Germany_2018.xodr");

  // unsigned int lane_type = getLaneInfoFromInertialPos(manager, 6045, -2011, 0);

  // cout << "laneType is: " << lane_type << endl;

  return 0;
}


