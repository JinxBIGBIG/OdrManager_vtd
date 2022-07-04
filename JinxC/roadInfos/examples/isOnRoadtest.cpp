#include "OdrManager.hh"

#include <iostream>
using namespace std;

bool isOnRoadMarkTest(const Point& p) {
  OpenDrive::Position* pos = m_manager.createPosition();
  m_manager.activatePosition(pos);

  m_manager.setInertialPos(p.x, p.y, p.z);
  bool result = m_manager.inertial2lane();

  // TODO:it's better to get the width of roadmark from opendrive file or vtd 
  const double roadmarkWidth = 0.12;

  if (result) {
    OpenDrive::RoadHeader* header = m_manager.getRoadHeader();
    double laneWidth = m_manager.getLaneWidth();
    cout << "laneWidth is" << laneWidth << endl;
    OpenDrive::LaneCoord posLane = m_manager.getLanePos();
    double offsetFromLaneCenter = posLane.getOffset();
    cout << "offsetFromLaneCenter is "<< offsetFromLaneCenter << endl;
    double dist = laneWidth / 2 - fabs(offsetFromLaneCenter);
    cout << "dist is " << dist << endl;
    if (dist < roadmarkWidth / 2)
      return true;
    else
      return false;
  }
   cout << "end" << endl;
  return result;
}

int main(int argc, char** argv) {
   cout << "Is On Road test" << endl;
    OpenDrive::OdrManager manager;
  // bool flag = true;
  bool flag = manager.loadFile("D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr");
  if (flag)
    cout << "load file successful." << endl;
  else
    cout << "load file wrong." << endl;
  bool isOnRoadmark = isOnRoadMarkTest(p1);
  if (isOnRoadmark)
    std::cout << "this point is on roadmark." << std::endl;
  else
    std::cout << "this point is not on roadmark." << std::endl;
  return 0;
}

