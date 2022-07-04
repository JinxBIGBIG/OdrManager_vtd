#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "posInfo.hpp"

#include <iostream>
#include <cmath>

using namespace std;
using namespace OpenDrive;

getPosInfo::getPosInfo() {}

bool getPosInfo::init(const std::string &odrFile)
{
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

bool getPosInfo::isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType)
{
  getInertialPosInfo(p);
  std::cout << "m_posInfo.RoadID:" <<m_posInfo.roadId<< std::endl;
  std::cout << "m_posInfo.SectionS:" <<m_posInfo.sectionS<< std::endl;
  std::cout << "m_posInfo.laneId:" <<m_posInfo.laneId<< std::endl;
  std::cout << "m_posInfo.laneType:" <<m_posInfo.laneType<< std::endl;
  std::cout << "roadType:" <<roadType<< std::endl;
  if (m_posInfo.laneType == roadType)
    return true;
  return false;
}

bool getPosInfo::isSameDirectionWithRoad(Point p,float heading1)
{
  getInertialPosInfo(p);
  float pi=atan(1)*4;
  float CarHeading=m_posInfo.roadheading-heading1 ;
   if (((pi/2) > heading1 > 0) || ((-pi/2) < heading1 < 0))
  {
    return 1;
  }
  return 0;
}

// TODO The speed limit signs
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

// TODO The speed limit signs
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

// TODO The speed limit signs
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

// TODO The speed limit signs
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

// TODO isTurning
bool getPosInfo::isInSce2_46_7(Point p)
{
  bool isInRoad = getInertialPosInfo(p);
  double pi=atan(1)*4;
  double headingChangeMin = (pi/36);
  {
    double slength=m_posInfo.roadlength;
    OpenDrive::OdrManager Manager;
    OpenDrive::Position* pos1 = Manager.createPosition();
    Manager.activatePosition(pos1);
    Manager.setLanePos(m_posInfo.roadId, m_posInfo.laneId, 0);
    Manager.lane2inertial();
    double p1x=Manager.getInertialPos().getX();
    double p1y=Manager.getInertialPos().getY();
    double p1z=Manager.getInertialPos().getZ();
    OpenDrive::GeoCoord myCoord(p1x, p1y, p1z);
    Manager.setPos( myCoord );
    double startH=myCoord.getH();
    posInfo pos_info1 = m_posInfo;
    while (pos_info1.section->getRight())
    {         
      OpenDrive::LaneSection *secInfo1 = pos_info1.section;
      OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
      pos_info1.section = nextSecInfo1;
      pos_info1.sectionS=pos_info1.section->mS;
      if (pos_info1.lane->getSuccessorLink())
      {
        pos_info1.lane = pos_info1.lane->getSuccessor();
        pos_info1.laneId = pos_info1.lane->mId;
      }
      else
      {
        break ;
      }
    }
    OpenDrive::OdrManager myManager;
    OpenDrive::Position* pos2 = myManager.createPosition();
    myManager.activatePosition(pos2);
    myManager.setLanePos(m_posInfo.roadId, pos_info1.lane->mId,slength);
    myManager.lane2inertial();
    double p2x=myManager.getInertialPos().getX();
    double p2y=myManager.getInertialPos().getY();
    double p2z=myManager.getInertialPos().getZ();
    OpenDrive::GeoCoord myCoord2(p2x, p2y, p2z);
    myManager.setPos( myCoord2 );
    const OpenDrive::GeoCoord &myCoord22=myCoord2;
    double endH = myCoord22.getH();
    if ((endH==0)&&(startH==0))
    {
      double headingChange = atan((p2x-p1x)/(p2y-p1y));
      if ((abs(headingChange)) > headingChangeMin)
      {
        return 1;
      }
    }
    else if (startH != endH )
    {
      return 1;
    }
  }
  return 0;
}

// TODO isDrivingDownSteep
bool getPosInfo::isInSce2_46_8(Point p,float heading1)
{
 bool isInRoad = getInertialPosInfo(p);
 bool alongRoadFlag= getPosInfo::isSameDirectionWithRoad(p,heading1);
 OpenDrive::Elevation* myElevation =m_posInfo.posElevation;
 double elvLength=(m_posInfo.ElvmSEnd)-(m_posInfo.ElvmS);
 double elvHeight=m_posInfo.ElvmB*elvLength+m_posInfo.ElvmC *elvLength*elvLength+m_posInfo.ElvmD*elvLength*elvLength*elvLength;
 int hIcreaseFlag=0;
 double slopeMin=0.07;
 if ((abs(elvHeight/elvLength))>slopeMin)
 {
   float sChange= (elvLength)/4 ;
   float hChange=m_posInfo.ElvmB*sChange+m_posInfo.ElvmC*sChange*sChange+m_posInfo.ElvmD*sChange*sChange*sChange;
   if (hChange > 0)
   {
     hIcreaseFlag=1;
   }
   else if (hChange < 0)
   {
     hIcreaseFlag=2;
   }
 }
 if ((alongRoadFlag==1 && hIcreaseFlag==2)||(alongRoadFlag==0 && hIcreaseFlag==1))
 {
   return 1 ;
 }
 return 0 ;
}

// TODO isTopRam
bool getPosInfo::isInSce2_59_2(Point p)
{
  getInertialPosInfo(p);
  if ((m_posInfo.ElvmA==0)&&(m_posInfo.ElvmB==0)&&(m_posInfo.ElvmC==0)&&(m_posInfo.ElvmD==0))
  {
    return 0;
  }
  else
  {
    float sPos = m_posInfo.trackS-m_posInfo.ElvmS;
    float d2s=m_posInfo.ElvmB+2*m_posInfo.ElvmC*sPos;
    if (d2s<0)
    {
      return 1;
    }
    else if ((d2s>=0) && (d2s<0.1))
    {
      float sChangeLeft= ( m_posInfo.trackS-m_posInfo.ElvmS)/10;
      float sChangeRight= ( m_posInfo.ElvmSEnd-m_posInfo.trackS)/10;
      int sPosI=1;
      float d2sRight=2*m_posInfo.ElvmC+6*m_posInfo.ElvmD*(sPos+sChangeRight*sPosI);
      float d2sLeft=2*m_posInfo.ElvmC+6*m_posInfo.ElvmD*(sPos-sChangeLeft*sPosI);
      if (((d2sRight<0)&&(d2sLeft<=0))||((d2sRight=0)&&(d2sLeft<0)))
      {
        return 1;
      }
      while ((d2sRight==0)&&(d2sLeft==0))
      {
        sPosI=sPosI+1;
        d2sRight=2*m_posInfo.ElvmC+6*m_posInfo.ElvmD*(sPos+sChangeRight*sPosI);
        d2sLeft=2*m_posInfo.ElvmC+6*m_posInfo.ElvmD*(sPos-sChangeLeft*sPosI);
        if (((d2sRight<0.01)&&(d2sLeft<=0))||((d2sRight=0)&&(d2sLeft<0.01)))
        {
          sPosI=1;
          return 1;
          break ;
        }
        else if ((d2sRight>0.1)||(d2sLeft>0.1))
        {
          sPosI=1;
          break ;
        }
        else if (sPosI>10)
        {
          
          break ;
        }
      }
    }

  }
  
  return 0;
}


posInfo getPosInfo::getInfo() const
{
  return m_posInfo;
}

