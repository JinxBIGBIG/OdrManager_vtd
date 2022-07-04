#include "posInfo.hpp"
#include "OdrManager.hh"
#include "OpenDRIVE.hh"

#include <cmath>
#include <iostream>

using namespace std;
using namespace OpenDrive;
getPosInfo::getPosInfo() {}

bool getPosInfo::init(const std::string& odrFile) {
    bool res = m_manager.loadFile(odrFile);
    return res;
}

// TODO: if the point is not in any road
bool getPosInfo::getInertialPosInfo(Point p)
{

  OpenDrive::Position *pos = m_manager.createPosition();
  m_manager.activatePosition(pos);

  m_manager.setInertialPos(p.x, p.y, p.z);
  bool result = m_manager.inertial2lane();

  if (result)
  {
    // OpenDrive::OdrManager myManager;
    // OpenDrive::GeoCoord myCoord(p.x, p.y, p.z);
    // myManager.setPos( myCoord );
    // m_posInfo.roadheading = myCoord.getH();

    OpenDrive::RoadHeader *header = m_manager.getRoadHeader();
    if (!header)
      return false;
    m_posInfo.roadId = header->getId();
    m_posInfo.road = header;
    m_posInfo.roadlength = header->mLength;

    OpenDrive::Lane *lane = m_manager.getLane();
    if (!lane)
      return false;
    m_posInfo.lane = lane;

    double speed = m_manager.getLaneSpeed();
    m_posInfo.laneId = lane->getId();
    m_posInfo.laneSpeed = speed;
    m_posInfo.laneType = lane->getType();

    OpenDrive::LaneSection *section = m_manager.getLaneSection();
    m_posInfo.section = section;
    if (!section)
      return false;
    m_posInfo.sectionS = section->mS;

    m_posInfo.junctionId = m_manager.getJunctionId();

    OpenDrive::TrackCoord posTrack = m_manager.getTrackPos();
    m_posInfo.trackS = posTrack.getS();
    m_posInfo.trackT = posTrack.getT();

    OpenDrive::Elevation* posElevation =m_manager.getElevation();
    m_posInfo.ElvmS=posElevation->mS;
    m_posInfo.ElvmSEnd=posElevation->mSEnd;
    m_posInfo.ElvmA=posElevation->mA;
    m_posInfo.ElvmB=posElevation->mB;
    m_posInfo.ElvmC=posElevation->mC;
    m_posInfo.ElvmD=posElevation->mD;
  }
  else
  {
    std::cout << "the point is not on road." << std::endl;
  }
  return result;
}

// // suppose the two points are not far， at most in two adjacent road
// // TODO: how to deal with points not in adjacent road
// //jiangmang
// int getPosInfo::isInSameLane(Point p1, Point p2) {
//   bool isInRoad = getInertialPosInfo(p1);
//   if (!isInRoad) return -1;
//   posInfo pos_info1 = m_posInfo;
//   isInRoad = getInertialPosInfo(p2);
//   posInfo pos_info2 = m_posInfo;
//   if (!isInRoad) return -1;

//   // check if in same road
//   if (pos_info1.roadId == pos_info2.roadId) {
//     // check if in same laneSection
//     //???
//     if (fabs(pos_info1.sectionS - pos_info2.sectionS) < 1e-5) {

//       if (pos_info1.laneId == pos_info2.laneId)
//         return 1;
//     } else if (pos_info1.sectionS < pos_info2.sectionS) {
//       if (pos_info1.lane->getSuccessor() &&
//       pos_info1.lane->getSuccessor()->getId() == pos_info2.lane->getId())
//         return 2;
//     } else {
//       if (pos_info2.lane->getSuccessor() &&
//       pos_info2.lane->getSuccessor()->getId() == pos_info1.lane->getId())
//         return 3;
//     }
// } else if (pos_info1.road->getSuccessorLink()
//            && pos_info1.road->getSuccessorLink()->getElemType() ==
//            OpenDrive::EnType::ODR_TYPE_ROAD
//            && pos_info1.road->getSuccessorLink()->getElemId() ==
//            pos_info2.road->getId()){
//     if (pos_info1.lane->getSuccessor() &&
//     pos_info1.lane->getSuccessor()->getId() == pos_info2.lane->getId())
//       return 4;
//   } else if (pos_info2.road->getSuccessorLink()
//              && pos_info2.road->getSuccessorLink()->getElemType() ==
//              OpenDrive::EnType::ODR_TYPE_ROAD
//              && pos_info2.road->getSuccessorLink()->getElemId() ==
//              pos_info1.road->getId()){
//     if (pos_info2.lane->getSuccessor() &&
//     pos_info2.lane->getSuccessor()->getId() == pos_info1.lane->getId())
//       return 5;
//   } else if (pos_info1.road->getPredecessorLink()
//              && pos_info1.road->getPredecessorLink()->getElemType() ==
//              OpenDrive::EnType::ODR_TYPE_ROAD
//              && pos_info1.road->getPredecessorLink()->getElemId() ==
//              pos_info2.road->getId()){
//     if (pos_info1.lane->getPredecessor() &&
//     pos_info1.lane->getPredecessor()->getId() == pos_info2.lane->getId())
//       return 6;
//   } else if (pos_info2.road->getPredecessorLink()
//              && pos_info2.road->getPredecessorLink()->getElemType() ==
//              OpenDrive::EnType::ODR_TYPE_ROAD
//              && pos_info2.road->getPredecessorLink()->getElemId() ==
//              pos_info1.road->getId()){
//     if (pos_info2.lane->getPredecessor() &&
//     pos_info2.lane->getPredecessor()->getId() == pos_info1.lane->getId())
//       return 7;
//   }

//   return 0;
// }
////
// int getPosInfo::drivingDirRelToRoad(bool isRHT) {
//   if (m_posInfo.laneId < 0 && isRHT)
//     return 1;
//   if (m_posInfo.laneId > 0 && !isRHT)
//     return 1;
//   return -1;
// }

// int getPosInfo::isInSameLane(Point p1, Point p2, bool isInFront) {
//   getInertialPosInfo(p1);
//   posInfo pos_info1 = m_posInfo;
//   int dir = drivingDirRelToRoad(true);
//   if (dir == 1) {
//     OpenDrive::RoadHeader* nexRoad = pos_info1.road->getSuccessor();
//     pos_info1.road->getSuccessorLink();
//   } else if (dir == -1) {
//     OpenDrive::RoadHeader* nexRoad = pos_info1.road->getPredecessor();
//   }
//   getInertialPosInfo(p2);
//   posInfo pos_info2 = m_posInfo;
//   return 0;
// }
// suppose the two points are not far
// TODO: how to deal with points not in adjacent road
// gx
/* int getPosInfo::Preprocess(Point p1,float heading1)
{
  float pi=4*atan(1);
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
    return -1;
  posInfo pos_info = m_posInfo;

  //if junction
  if (((pos_info.roadlength-pos_info.trackS<20)
  &&(pos_info1.road->getSuccessor())
  &&(pos_info1.road->getSuccessorLink()->getElemType() ==
OpenDrive::EnType::ODR_TYPE_JUNCTION)))
  {
    if (pos_info1.road->getSuccessorLink().mIncomingRoad == pos_info1.road)
    {
      pos_info1.road=pos_info1.road->getSuccessorLink().mConnectingRoad;
    }



  }
}
 */

bool getPosInfo::isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType) {
    getInertialPosInfo(p);
    if (m_posInfo.laneType == roadType)
        return true;
    return false;
}

// bool getPosInfo::isOnRoadMark(const Point& p) {
//   OpenDrive::Position* pos = m_manager.createPosition();
//   m_manager.activatePosition(pos);

//   m_manager.setInertialPos(p.x, p.y, p.z);
//   bool result = m_manager.inertial2lane();

//   // TODO:it's better to get the width of roadmark from opendrive file or vtd
//   const double roadmarkWidth = 0.12;

//   if (result) {
//     OpenDrive::RoadHeader* header = m_manager.getRoadHeader();
//     double laneWidth = m_manager.getLaneWidth();
//     OpenDrive::LaneCoord posLane = m_manager.getLanePos();
//     double offsetFromLaneCenter = posLane.getOffset();
//     double dist = laneWidth / 2 - fabs(offsetFromLaneCenter);
//     if (dist < roadmarkWidth / 2)
//       return true;
//     else
//       return false;
//   }
//   return result;
// }

posInfo getPosInfo::getInfo() const {
    return m_posInfo;
}

// return 1 means increasing dir along road s
//       -1 means decreasing dir of road s
// int getPosInfo::drivingDirRelToRoad(bool isRHT) {
//   if (m_posInfo.laneId < 0 && isRHT)
//     return 1;
//   if (m_posInfo.laneId > 0 && !isRHT)
//     return 1;
//   return -1;
// }
//!==============================================================================================================
// TODO 限速标线
bool getPosInfo::isInSce2_45_6(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_TOWN) {
        return false;
    }

    RoadHeader* myRoad = m_posInfo.road;
    Signal* mysignal = myRoad->getFirstSignal();
    while (mysignal) {
        if (mysignal->mType == 274) {
            return false;
        }
        mysignal = (Signal*)mysignal->getRight();
    }
    Lane* midLane = m_manager.getLaneSection()->getLaneFromId(0);
    if (midLane->getFirstRoadMark() && midLane->getFirstRoadMark()->getType() != ODR_ROAD_MARK_TYPE_NONE) {
        return false;
    }
    return true;
}

// TODO 限速标线/vtd中目前显示type为highway的道路在xodr文件中是rural，因此暂定rural代指公路
bool getPosInfo::isInSce2_45_7(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_RURAL) {
        return false;
    }

    RoadHeader* myRoad = m_posInfo.road;
    Signal* mysignal = myRoad->getFirstSignal();
    while (mysignal) {
        if (mysignal->mType == 274) {
            return false;
        }
        mysignal = (Signal*)mysignal->getRight();
    }
    Lane* midLane = m_manager.getLaneSection()->getLaneFromId(0);
    if (midLane->getFirstRoadMark() && midLane->getFirstRoadMark()->getType() != ODR_ROAD_MARK_TYPE_NONE) {
        return false;
    }
    return true;
}

// TODO 限速标线
bool getPosInfo::isInSce2_45_8(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_TOWN) {
        return false;
    }

    RoadHeader* myRoad = m_posInfo.road;
    Signal* mysignal = myRoad->getFirstSignal();
    while (mysignal) {
        if (mysignal->mType == 274) {
            return false;
        }
        mysignal = (Signal*)mysignal->getRight();
    }

    Lane* myLane = m_manager.getLane();
    if (myLane->getType() != ODR_LANE_TYPE_DRIVING) {
        return false;
    }
    int ID = myLane->getId();
    Lane* myfind = myLane;
    while (myfind->getLeft()) {
        Lane* leftLane = (Lane*)(myfind->getLeft());
        if (leftLane->getId() * ID <= 0)
            break;
        if (leftLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = leftLane;
    }

    myfind = myLane;
    while (myfind->getRight()) {
        Lane* rightLane = (Lane*)(myfind->getRight());
        if (rightLane->getId() * ID <= 0)
            break;
        if (rightLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = rightLane;
    }
    return true;
}

// TODO 限速标线
bool getPosInfo::isInSce2_45_9(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_RURAL) {
        return false;
    }

    RoadHeader* myRoad = m_posInfo.road;
    Signal* mysignal = myRoad->getFirstSignal();
    while (mysignal) {
        if (mysignal->mType == 274) {
            return false;
        }
        mysignal = (Signal*)mysignal->getRight();
    }

    Lane* myLane = m_manager.getLane();
    if (myLane->getType() != ODR_LANE_TYPE_DRIVING) {
        return false;
    }
    int ID = myLane->getId();
    Lane* myfind = myLane;
    while (myfind->getLeft()) {
        Lane* leftLane = (Lane*)(myfind->getLeft());
        if (leftLane->getId() * ID <= 0)
            break;
        if (leftLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = leftLane;
    }

    myfind = myLane;
    while (myfind->getRight()) {
        Lane* rightLane = (Lane*)(myfind->getRight());
        if (rightLane->getId() * ID <= 0)
            break;
        if (rightLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = rightLane;
    }
    return true;
}