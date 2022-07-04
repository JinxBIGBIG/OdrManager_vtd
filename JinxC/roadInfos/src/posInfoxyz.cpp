#include "posInfoxyz.hpp"
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

// TODO: 同方向是否只有一条机动车道
// 0:不在道路中或者左侧右侧车道中 ；1 ：一条机动车道； 2：多条机动车道
int getPosInfo::isInSce3_9_1(Point p){
    bool isInRoad = getInertialPosInfo(p);
    if(!isInRoad){
        return 0;
    }
    if (m_manager.getRoadType()==ODR_ROAD_TYPE_NONE || m_manager.getRoadType()==ODR_ROAD_TYPE_PEDESTRIAN || m_manager.getRoadType()==ODR_ROAD_TYPE_BICYCLE ) {
        return 0;
    }
    
    RoadHeader* header = m_posInfo.road;
    cout <<"road id: "<< header->getId()<<endl;
    LaneSection* ls = m_posInfo.section;
    // cout <<"lane section: " << ls->mS <<endl;

    Lane* tempLane = m_posInfo.lane;
    Lane* tempLane2 = m_posInfo.lane;
    int tempLaneID0 = m_posInfo.laneId;
    int tempLaneID = m_posInfo.laneId; 
    int tempLaneID2 = m_posInfo.laneId; 
    cout<<"laneID:" << tempLaneID <<endl;

    unsigned int tempLaneType = tempLane->getType();
    unsigned int tempLaneType2 = tempLane->getType();
    cout<<"lane type:" << tempLaneType <<endl;

    if(tempLaneID0==0){
        return 0;
    }
    
    int finalLaneID = 0;  // outermost id of lane
    if (tempLaneType==ODR_LANE_TYPE_DRIVING){
        finalLaneID = 1;
    }
    if(tempLaneID0>0)
    {   
        while(tempLane!=NULL){
            tempLane = reinterpret_cast<Lane*>(tempLane->getLeft());
            tempLaneType = tempLane->getType();
            cout <<"a"<<endl;
            // if (tempLaneType==ODR_LANE_TYPE_DRIVING){
            if (tempLaneType==6){
                finalLaneID = finalLaneID+1; 
                cout<<"finalLaneID:" <<finalLaneID<<endl;
                cout<<"aaaaaaaaaaaaaaaaaaaa:"<< tempLane->getId()<<endl;
            }
        }
        cout<<"bbbbbbbbbbbbbbbbb" <<endl;
    }
    cout<<"start_tempLaneID2:"<< tempLaneID2 <<endl;
    if(tempLaneID2>0)
    {
        cout<<"start_tempLaneID2:"<< tempLaneID2 <<endl;
        while(tempLaneID2!=0){
            
            tempLane2 = reinterpret_cast<Lane*>(tempLane2->getRight());
            tempLaneID2 = tempLane2->getId();
            cout<<"tempLaneID2:"<< tempLaneID2 <<endl;
            tempLaneType2 = tempLane2->getType();
            if (tempLaneType2==ODR_LANE_TYPE_DRIVING){
                finalLaneID = finalLaneID+1;  
                cout << "findLaneID: " << finalLaneID <<endl;
            }
        }
        
    }

    if(tempLaneID<0)
    {
        while (tempLane!=NULL)
        {
            tempLane = reinterpret_cast<Lane*>(tempLane->getRight());
            tempLaneType = tempLane->getType();
            if (tempLaneType==ODR_LANE_TYPE_DRIVING){
                finalLaneID = finalLaneID+1;  
            }
        }
        while (tempLaneID2!=0)
        {
            tempLane2 = reinterpret_cast<Lane*>(tempLane2->getLeft());
            tempLaneID2 = tempLane2->getId();
            tempLaneType2 = tempLane2->getType();
            if (tempLaneType2==ODR_LANE_TYPE_DRIVING){
                finalLaneID = finalLaneID+1;  
            }
        }
        
    }
    
    if (finalLaneID==1){
        return 1;
    }
    else if(finalLaneID==2){
        return 2;
    }
    else{
        return 0;
    }
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


