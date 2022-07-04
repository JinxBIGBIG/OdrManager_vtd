#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "posInfo.hpp"

#include <iostream>
#include <cmath>

using namespace std;
using namespace OpenDrive;

getPosInfo::getPosInfo() {}

//jm
bool getPosInfo::init(const std::string &odrFile)
{
  bool res = m_manager.loadFile(odrFile);
  return res;
}

// TODO: if the point is not in any road：jm/gx
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
    // OpenDrive::OdrManager myManager;
    // OpenDrive::Position* pos = myManager.createPosition();
    // myManager.activatePosition(pos);
    
  

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
    m_posInfo.laneWidth=m_manager.getLaneWidth();

    OpenDrive::LaneSection *section = m_manager.getLaneSection();
    m_posInfo.section = section;
    if (!section)
      return false;
    m_posInfo.sectionS = section->mS;

    m_posInfo.junctionId = m_manager.getJunctionId();

    OpenDrive::TrackCoord posTrack = m_manager.getTrackPos();
    m_posInfo.trackS = posTrack.getS();
    m_posInfo.trackT = posTrack.getT();
    m_posInfo.roadheading= posTrack.getH();


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

//gx
int** getPosInfo::GetJuncDir(Point p1,float heading1)
{
  int** a;
  a=(int **)malloc(10*sizeof(int *));
  for (int ai=0;ai<10;ai++)
  {
		a[ai]=(int *)malloc(2*sizeof(int));
  }
  int ai=0;

  int bi=0;

  float pi=4*atan(1);
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
  {
    a[0][0]=-1;
    return a;
  }
  posInfo pos_info1 = m_posInfo;
  pos_info1.roadId=pos_info1.road->mId;
  pos_info1.laneId=pos_info1.lane->mId;
  // // roadheading = myCoord.getH();
  // std::cout << "Preprocess enable :pos_info1.roadId is " <<pos_info1.roadId<<std::endl;
  // std::cout << "Preprocess enable :pos_info1.laneId is " <<pos_info1.laneId<<std::endl;
  // // //if junction
  // // if (((pos_info1.roadlength-pos_info1.trackS<20)
  // // &&(pos_info1.road->getSuccessor())
  // // &&(pos_info1.road->getSuccessorLink()->getElemType() == OpenDrive::EnType::ODR_TYPE_ROAD)))
  // //根据junction里面的连接关系，得到与自车lane相连接的road及lane
  if ((pos_info1.road->getSuccessor(1))&&(pos_info1.road->getSuccessorLink()==NULL))
  {
    OpenDrive::RoadHeader* myroad = pos_info1.road->getSuccessor(1);
    OpenDrive::JuncHeader *myJunction = reinterpret_cast<OpenDrive::JuncHeader *>(myroad->getJunction());
    OpenDrive::JuncLink *myJuncLink =reinterpret_cast<OpenDrive::JuncLink *>(myJunction->getFirstLink());
    int junctionId=myJunction->mId;
    int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
    int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
    int mConnectDir= 0;
    if (myJuncLink->mDir==0)
    {
      mConnectDir= 10;
    }
    else
    {
      mConnectDir= 11;
    }
    OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
    // std::cout << " myroadid" <<pos_info1.roadId<<std::endl;
    // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
    // std::cout << " mConnectRoadId" <<mConnectRoadId<<std::endl;
    // std::cout << " mConnectLaneId" <<myLaneLink->mConnectingLane->mId<<std::endl;
    if (mIncomRoadId == pos_info1.road->mId)
    {
      // a[0][0]=mConnectRoadId;
      a[0][0]=10;
      // std::cout << " 1myroadid" <<pos_info1.roadId<<std::endl;
      // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
      if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
      {
        // a[0][1]=myLaneLink->mConnectingLane->mId;
        a[0][1]=mConnectDir;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          // std::cout << " 2myroadid" <<pos_info1.roadId<<std::endl;
          // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
          if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
          {
            // a[0][1]=myLaneLink->mConnectingLane->mId;
            a[0][1]=mConnectDir;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
      std::cout << " mConnectRoadId" <<mConnectRoadId<<std::endl;
    }
    else if (mConnectRoadId == pos_info1.road->mId)
    {
      // a[0][0]=mIncomRoadId;
      a[0][0]=11;
      // std::cout << " 3myroadid" <<pos_info1.roadId<<std::endl;
      // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
      if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
      {
        // a[0][1]=myLaneLink->mIncomingLane->mId;
        a[0][1]=mConnectDir;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          // std::cout << " 4myroadid" <<pos_info1.roadId<<std::endl;
          // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
          if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
          {
            // a[0][1]=myLaneLink->mIncomingLane->mId;
            a[0][1]=mConnectDir;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
      // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
    }
    while  (myJuncLink->getRight()!=NULL)
    {
      OpenDrive::JuncLink *nextJuncLink = (OpenDrive::JuncLink *)myJuncLink->getRight();
      myJuncLink = nextJuncLink;
      OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
      int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
      int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
      // std::cout << " 2mIncomRoadId" <<mIncomRoadId<<std::endl;
      // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl; 
      // //如何获取road的heading
      // // pos_info1.road=pos_info1.road->getH();
      // // const double&GeoCoord::getH();
      // // pos_info1.roadheading = pos_info1.road->getH();
      // // if (mIncomRoadId == pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
      if (mIncomRoadId == pos_info1.road->mId)
      {
        // std::cout << " 5myroadid" <<pos_info1.roadId<<std::endl;
        // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
        // std::cout << " myLaneLink->mIncomingLane->mId" <<myLaneLink->mIncomingLane->mId<<std::endl;
        // a[ai][0]=mConnectRoadId;
        a[ai][0]=10;
        if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
        {
          // a[ai][1]=myLaneLink->mConnectingLane->mId;
          a[ai][1]=mConnectDir;
          // std::cout << " 5gxmyroadid" <<std::endl;
        }
        else 
        {
          std::cout << " 55myroadid" <<std::endl;
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            // std::cout << " 566myLaneLink->mConnectingLane->mId:" <<std::endl;
            // std::cout << " 566myLaneLink->mConnectingLane->mId:" <<myLaneLink->mConnectingLane->mId<<std::endl;
            if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
            {
              // a[ai][1]=myLaneLink->mConnectingLane->mId;
              a[ai][1]=mConnectDir;
            }
            if (myLaneLink->getRight()==NULL)
            {
              // std::cout << " 56myroadid" <<pos_info1.roadId<<std::endl;
              break;
            }
          }
        }
        ai=ai+1;
        // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl;
      }
      // else if (mConnectRoadId == pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
      else if (mConnectRoadId == pos_info1.road->mId)
      {
        // std::cout << " 6myroadid" <<pos_info1.roadId<<std::endl;
        // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;  
        // a[ai][0]=mIncomRoadId;
        a[ai][0]=11;
        if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
        {
          a[ai][1]=mConnectDir;
          // a[ai][1]=myLaneLink->mIncomingLane->mId;
        }
        else 
        {
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
            {
              // a[ai][1]=myLaneLink->mIncomingLane->mId;
              a[ai][1]=mConnectDir;
            }
            if (myLaneLink->getRight()==NULL)
            {
              break;
            }
          }
        }
        ai=ai+1;
        // std::cout << " 2mIncomRoadId" <<mConnectRoadId<<std::endl;
      }
      if (myJuncLink->getRight()==NULL)
      {
        break;
      }
    }
    //当点1所在lane与多个路的lane有连接时，根据自车与车道线的夹角进行筛选,若夹角大于30度，过滤掉该结果；
    if (a[2][1])
    {
      //如何获取road的heading
      // std::cout << " shaixuan" <<std::endl;
      OpenDrive::OdrManager myManager;
      OpenDrive::Position* pos = myManager.createPosition();
      myManager.activatePosition(pos);
      OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
      myManager.setPos( myCoord );
      bool result = myManager.lane2inertial();
      pos_info1.roadheading=myCoord.getH();
      int aiMax=ai ;
      for (ai=0;ai<=aiMax;ai++)
      {
        if ((pos_info1.roadheading-heading1)<(pi/6)&&(a[ai][1]))
        {
          a[bi][0]=a[ai][0];
          a[bi][1]=a[ai][1];
          bi=bi+1;
        }
        // std::cout << " bi " <<bi<<std::endl;
      }
      // std::cout << " shaixuanjieshu " <<pos_info1.roadheading<<std::endl;
    }
    ai=0;   
    bi=0;   
    // std::cout << " a" <<a<<std::endl;
  }
  return a;
}

//gx
int** getPosInfo::Preprocess(Point p1,float heading1)
// int* getPosInfo::Preprocess()
{
  int** a;
  a=(int **)malloc(10*sizeof(int *));
  for (int ai=0;ai<10;ai++)
  {
		a[ai]=(int *)malloc(2*sizeof(int));
  }
  int ai=0;

  int bi=0;

  float pi=4*atan(1);
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
  {
    a[0][0]=-1;
    return a;
  }
  posInfo pos_info1 = m_posInfo;
  pos_info1.roadId=pos_info1.road->mId;
  pos_info1.laneId=pos_info1.lane->mId;
  // // roadheading = myCoord.getH();
  // std::cout << "Preprocess enable :pos_info1.roadId is " <<pos_info1.roadId<<std::endl;
  // std::cout << "Preprocess enable :pos_info1.laneId is " <<pos_info1.laneId<<std::endl;
  // // //if junction
  // // if (((pos_info1.roadlength-pos_info1.trackS<20)
  // // &&(pos_info1.road->getSuccessor())
  // // &&(pos_info1.road->getSuccessorLink()->getElemType() == OpenDrive::EnType::ODR_TYPE_ROAD)))
  // //根据junction里面的连接关系，得到与自车lane相连接的road及lane
  if ((pos_info1.road->getSuccessor(1))&&(pos_info1.road->getSuccessorLink()==NULL))
  {
    OpenDrive::RoadHeader* myroad = pos_info1.road->getSuccessor(1);
    OpenDrive::JuncHeader *myJunction = reinterpret_cast<OpenDrive::JuncHeader *>(myroad->getJunction());
    OpenDrive::JuncLink *myJuncLink =reinterpret_cast<OpenDrive::JuncLink *>(myJunction->getFirstLink());
    int junctionId=myJunction->mId;
    int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
    int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
    OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
    // std::cout << " myroadid" <<pos_info1.roadId<<std::endl;
    // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
    // std::cout << " mConnectRoadId" <<mConnectRoadId<<std::endl;
    // std::cout << " mConnectLaneId" <<myLaneLink->mConnectingLane->mId<<std::endl;
    if (mIncomRoadId == pos_info1.road->mId)
    {
      a[0][0]=mConnectRoadId;
      std::cout << " 1myroadid" <<pos_info1.roadId<<std::endl;
      std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
      if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
      {
        a[0][1]=myLaneLink->mConnectingLane->mId;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          // std::cout << " 2myroadid" <<pos_info1.roadId<<std::endl;
          // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
          if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
          {
            a[0][1]=myLaneLink->mConnectingLane->mId;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
      std::cout << " mConnectRoadId" <<mConnectRoadId<<std::endl;
    }
    else if (mConnectRoadId == pos_info1.road->mId)
    {
      a[0][0]=mIncomRoadId;
      // std::cout << " 3myroadid" <<pos_info1.roadId<<std::endl;
      // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
      if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
      {
        a[0][1]=myLaneLink->mIncomingLane->mId;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          // std::cout << " 4myroadid" <<pos_info1.roadId<<std::endl;
          // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
          if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
          {
            a[0][1]=myLaneLink->mIncomingLane->mId;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
      // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
    }
    while  (myJuncLink->getRight()!=NULL)
    {
      OpenDrive::JuncLink *nextJuncLink = (OpenDrive::JuncLink *)myJuncLink->getRight();
      myJuncLink = nextJuncLink;
      OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
      int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
      int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
      // std::cout << " 2mIncomRoadId" <<mIncomRoadId<<std::endl;
      // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl; 
      //如何获取road的heading
      // pos_info1.road=pos_info1.road->getH();
      // const double&GeoCoord::getH();
      // pos_info1.roadheading = pos_info1.road->getH();
      // if (mIncomRoadId == pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
      if (mIncomRoadId == pos_info1.road->mId)
      {
        // std::cout << " 5myroadid" <<pos_info1.roadId<<std::endl;
        // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
        // std::cout << " myLaneLink->mIncomingLane->mId" <<myLaneLink->mIncomingLane->mId<<std::endl;
        a[ai][0]=mConnectRoadId;
        if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
        {
          a[ai][1]=myLaneLink->mConnectingLane->mId;
          // std::cout << " 5gxmyroadid" <<std::endl;
        }
        else 
        {
          // std::cout << " 55myroadid" <<std::endl;
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            // std::cout << " 566myLaneLink->mConnectingLane->mId:" <<std::endl;
            // std::cout << " 566myLaneLink->mConnectingLane->mId:" <<myLaneLink->mConnectingLane->mId<<std::endl;
            if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
            {
              a[ai][1]=myLaneLink->mConnectingLane->mId;
            }
            if (myLaneLink->getRight()==NULL)
            {
              // std::cout << " 56myroadid" <<pos_info1.roadId<<std::endl;
              break;
            }
          }
        }
        ai=ai+1;
        // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl;
      }
      // else if (mConnectRoadId == pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
      else if (mConnectRoadId == pos_info1.road->mId)
      {
        // std::cout << " 6myroadid" <<pos_info1.roadId<<std::endl;
        // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;  
        a[ai][0]=mIncomRoadId;
        if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
        {
          a[ai][1]=myLaneLink->mIncomingLane->mId;
        }
        else 
        {
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
            {
              a[ai][1]=myLaneLink->mIncomingLane->mId;
            }
            if (myLaneLink->getRight()==NULL)
            {
              break;
            }
          }
        }
        ai=ai+1;
        // std::cout << " 2mIncomRoadId" <<mConnectRoadId<<std::endl;
      }
      if (myJuncLink->getRight()==NULL)
      {
        break;
      }
    }
    //当点1所在lane与多个路的lane有连接时，根据自车与车道线的夹角进行筛选,若夹角大于30度，过滤掉该结果；
    if (a[2][1])
    {
      //如何获取road的heading
      std::cout << " shaixuan" <<std::endl;
      OpenDrive::OdrManager myManager;
      OpenDrive::Position* pos = myManager.createPosition();
      myManager.activatePosition(pos);
      OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
      myManager.setPos( myCoord );
      bool result = myManager.lane2inertial();
      pos_info1.roadheading=myCoord.getH();
      int aiMax=ai ;
      for (ai=0;ai<=aiMax;ai++)
      {
        if ((pos_info1.roadheading-heading1)<(pi/6)&&(a[ai][1]))
        {
          a[bi][0]=a[ai][0];
          a[bi][1]=a[ai][1];
          bi=bi+1;
        }
      }
      std::cout << " bi " <<bi-1<<std::endl;
      std::cout << " shaixuanjieshu: " <<pos_info1.roadheading<<std::endl;
    }
    ai=0;   
    bi=0;   
    // std::cout << " a" <<a<<std::endl;
  }
  return a;
}

//gx
int** getPosInfo::JuncLinkRoad(Point p1,float heading1)
// int* getPosInfo::Preprocess()
{
  int** a;
  a=(int **)malloc(10*sizeof(int *));
  for (int ai=0;ai<10;ai++)
  {
		a[ai]=(int *)malloc(2*sizeof(int));
  }
  int ai=0;

  int bi=0;

  float pi=4*atan(1);
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
  {
    a[0][0]=-1;
    return a;
  }
  posInfo pos_info1 = m_posInfo;
  pos_info1.roadId=pos_info1.road->mId;
  pos_info1.laneId=pos_info1.lane->mId;
  // //根据junction里面的连接关系，得到与自车lane相连接的road及lane
  if ((m_manager.getRoadHeader()->getJunctionNo()!=-1))
  {
    OpenDrive::RoadHeader* myroad = pos_info1.road;
    OpenDrive::JuncHeader *myJunction = reinterpret_cast<OpenDrive::JuncHeader *>(myroad->getJunction());
    OpenDrive::JuncLink *myJuncLink =reinterpret_cast<OpenDrive::JuncLink *>(myJunction->getFirstLink());
    int junctionId=myJunction->mId;
    int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
    int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
    OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
    if (mIncomRoadId == pos_info1.road->mId)
    {
      a[0][0]=mConnectRoadId;
      if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
      {
        a[0][1]=myLaneLink->mConnectingLane->mId;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
          {
            a[0][1]=myLaneLink->mConnectingLane->mId;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
    }
    else if (mConnectRoadId == pos_info1.road->mId)
    {
      a[0][0]=mIncomRoadId;
      if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
      {
        a[0][1]=myLaneLink->mIncomingLane->mId;
      }
      else 
      {
        while  (myLaneLink->getRight()!=NULL)
        {
          OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
          myLaneLink = nextJuncLaneLink;
          if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
          {
            a[0][1]=myLaneLink->mIncomingLane->mId;
          }
          if (myLaneLink->getRight()==NULL)
          {
            break;
          }
        }
      }
      ai=ai+1;
    }
    while  (myJuncLink->getRight()!=NULL)
    {
      OpenDrive::JuncLink *nextJuncLink = (OpenDrive::JuncLink *)myJuncLink->getRight();
      myJuncLink = nextJuncLink;
      OpenDrive::JuncLaneLink* myLaneLink=myJuncLink->getFirstLaneLink();
      int mIncomRoadId=myJuncLink->mIncomingRoad->mId;
      int mConnectRoadId=myJuncLink->mConnectingRoad->mId;
      if (mIncomRoadId == pos_info1.road->mId)
      {
        a[ai][0]=mConnectRoadId;
        if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
        {
          a[ai][1]=myLaneLink->mConnectingLane->mId;
        }
        else 
        {
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            if (myLaneLink->mIncomingLane->mId==pos_info1.lane->mId)
            {
              a[ai][1]=myLaneLink->mConnectingLane->mId;
            }
            if (myLaneLink->getRight()==NULL)
            {
              break;
            }
          }
        }
        ai=ai+1;
      }
      else if (mConnectRoadId == pos_info1.road->mId)
      {
        a[ai][0]=mIncomRoadId;
        if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
        {
          a[ai][1]=myLaneLink->mIncomingLane->mId;
        }
        else 
        {
          while  (myLaneLink->getRight()!=NULL)
          {
            OpenDrive::JuncLaneLink *nextJuncLaneLink = (OpenDrive::JuncLaneLink *)myLaneLink->getRight();
            myLaneLink = nextJuncLaneLink;
            if (myLaneLink->mConnectingLane->mId==pos_info1.lane->mId)
            {
              a[ai][1]=myLaneLink->mIncomingLane->mId;
            }
            if (myLaneLink->getRight()==NULL)
            {
              break;
            }
          }
        }
        ai=ai+1;
      }
      if (myJuncLink->getRight()==NULL)
      {
        break;
      }
    }
    //当点1所在lane与多个路的lane有连接时，根据自车与车道线的夹角进行筛选,若夹角大于30度，过滤掉该结果；
    if (a[2][1])
    {
      //如何获取road的heading
      std::cout << " shaixuan" <<std::endl;
      OpenDrive::OdrManager myManager;
      OpenDrive::Position* pos = myManager.createPosition();
      myManager.activatePosition(pos);
      OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
      myManager.setPos( myCoord );
      bool result = myManager.lane2inertial();
      pos_info1.roadheading=myCoord.getH();
      int aiMax=ai ;
      for (ai=0;ai<=aiMax;ai++)
      {
        if ((pos_info1.roadheading-heading1)<(pi/6)&&(a[ai][1]))
        {
          a[bi][0]=a[ai][0];
          a[bi][1]=a[ai][1];
          bi=bi+1;
        }
      }
      std::cout << " bi " <<bi-1<<std::endl;
      std::cout << " shaixuanjieshu: " <<pos_info1.roadheading<<std::endl;
    }
    ai=0;   
    bi=0;   
  }
  return a;
}

//gx
int getPosInfo::isInBackLane(Point p1, Point p2,float heading1,float *length)
{
  std::cout << "isInBackLane enable 1" << std::endl;
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
    return 0;
  posInfo pos_info1 = m_posInfo;
  isInRoad = getInertialPosInfo(p2);
  posInfo pos_info2 = m_posInfo;
  if (!isInRoad)
    return 0;
  int isInFrontLaneFlag = 0;
  int isInBackLaneFlag = 0;
  int LengthSumMax=150;
  double lengthSum=0;
  //check if in same road
  if (pos_info1.roadId == pos_info2.roadId)
  {
    std::cout << "isInSameRoad " << std::endl;
    // check if in same laneSection
    if (pos_info1.sectionS == pos_info2.sectionS)
    {
      std::cout << "isInSameSec " << std::endl;
      //the same road and the same lane
      if (pos_info1.laneId == pos_info2.laneId)
      {
        std::cout << "isInSamelane " << std::endl;
        if (pos_info1.trackS > pos_info2.trackS)
        {
          int isInBackLaneFlag = 1;
          std::cout << "in same lane isInBackLaneFlag :" << isInBackLaneFlag << endl;
          lengthSum =pos_info1.trackS-pos_info2.trackS;
          (*length)=lengthSum;
          return 11;
        }
      }
    }
    else if (pos_info1.sectionS > pos_info2.sectionS)
    {
      std::cout << "isInbacksec " << std::endl;
      bool endwhileFlag = 0;
      while ((pos_info1.roadId == pos_info2.roadId) && (pos_info1.sectionS < pos_info2.sectionS))
      {
        // pos_info2.sectionS=pos_info2.sectionS->getNextSections();
        OpenDrive::LaneSection *secInfo2 = pos_info2.section;
        OpenDrive::LaneSection *nextSecInfo2 = (OpenDrive::LaneSection *)secInfo2->getRight();
        pos_info2.section = nextSecInfo2;
        // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
        // pos_info2.laneId = laneInfo2-> mLaneId;
        pos_info2.laneId = pos_info2.lane->getSuccessor()->mId;
        if (pos_info1.section == pos_info2.section)
        {
          if (pos_info1.laneId == pos_info2.laneId)
          {
            lengthSum =pos_info1.trackS-pos_info2.trackS;
            (*length)=lengthSum;
            return 11;
            //2 is at the back of 1;
            int isInBackLaneFlag = 1;
            std::cout << "in same road not one section  isInBackLaneFlag :" << isInBackLaneFlag << endl;
          }
          else
          {
            return -1;
            //the same road but not the same lane
          }
          endwhileFlag = 1;
        }
        if (endwhileFlag == 1)
        {
          endwhileFlag = 0;
          break;
        }
      }
    }
  }
  // check if not in same road ,if in the Back road;
  std::cout << "isbackroad " << std::endl;
  bool endwhileFlag = 0;
  bool nextroadI = 0;
  posInfo pos_info1old = pos_info1;
  posInfo pos_info1old1 = pos_info1;
  pos_info1.roadlength = 0;
  int GetRoadSuccessFlag=10;
  // lengthSum = fabs(lengthSum) +  fabs(pos_info1.roadlength);
  while ((pos_info1.roadId != pos_info2.roadId) && (lengthSum < LengthSumMax))
  {
    // std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
    // pos_info1.roadlength = pos_info1.road->mLength;
    std::cout << "isBackroad " << std::endl;
    std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
    std::cout << "2roadlength " << pos_info1.roadlength << std::endl;
    if (((pos_info1.road->getSuccessorLink()!=NULL)&& GetRoadSuccessFlag==11)
        ||((pos_info1.road->getPredecessorLink()!=NULL)&& GetRoadSuccessFlag==10))
    {
      std::cout << "3getSuccessorLink " << std::endl;
      std::cout << "4pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
      std::cout << "5pos_info1.lane  " <<pos_info1.laneId << std::endl;
      bool nextRoadFlag = 0;
      //if have next sec
      {
        // if ((pos_info1old1.road->getSuccessorDir() == 0) && (nextroadI == 1))
        if ((GetRoadSuccessFlag==11) && (nextroadI == 1))
        {
          std::cout << "6getSuccessorLink " << std::endl;
         // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
          pos_info1old1.road = pos_info1old.road;
          pos_info1old.road = pos_info1.road;
          if (pos_info1old1.road->getSuccessorDir()==0)
          {
            GetRoadSuccessFlag=11;
          }
          else
          {
            GetRoadSuccessFlag=10;
          }
          pos_info1.road = pos_info1.road->getSuccessor();
          pos_info1.roadId = pos_info1.road->mId;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getSuccessorLink())
          {
            pos_info1.lane = pos_info1.lane->getSuccessor();
            pos_info1.laneId = pos_info1.lane->mId;
            std::cout << "7pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          if ((pos_info1old.road->getSuccessorDir() == 0) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.trackS;
                (*length) =lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "8pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.lane = pos_info1.lane->getSuccessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "9pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length) =lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
          else if ((pos_info1old.road->getSuccessorDir() == 1) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getLastLaneSection();
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length) =lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "10pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.lane = pos_info1.lane->getPredecessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "11pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length) =lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
        }

        // if ((pos_info1old1.road->getSuccessorDir() == 1) && (nextroadI == 1))
        if ((GetRoadSuccessFlag==10) && (nextroadI == 1))
        {
          std::cout << "12getSuccessorLink " << std::endl;
        // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
          pos_info1old1.road = pos_info1old.road;
          pos_info1old.road = pos_info1.road;
          if (pos_info1old1.road->getPredecessorDir()==0)
          {
            GetRoadSuccessFlag=11;
          }
          else
          {
            GetRoadSuccessFlag=10;
          }
          pos_info1.road = pos_info1.road->getPredecessor();
          pos_info1.roadId = pos_info1.road->mId;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getPredecessorLink())
          {
            pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
            std::cout << "13pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          if ((pos_info1old.road->getSuccessorDir() == 0) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.trackS;
                (*length) =lengthSum;
                return 11;
                // int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "14pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                std::cout << "15pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.lane==pos_info2.lane)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  // int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
          else if ((pos_info1old.road->getSuccessorDir() == 1) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getLastLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length) =lengthSum;
                return 11;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "16pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
                std::cout << "17pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.lane==pos_info2.lane)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  // int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
        }
        else if (nextroadI == 0)
        {
          std::cout << "18pos_info1.roadId " << pos_info1.roadId << std::endl;    
          while (pos_info1.section->getLeft())
          {           
            OpenDrive::LaneSection *secInfo1 = pos_info1.section;
            OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
            pos_info1.section = nextSecInfo1;
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "19pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
            // pos_info1.sectionS = nextSecInfo1->mS;
            if (pos_info1.lane->getPredecessorLink())
            {
              pos_info1.lane = pos_info1.lane->getPredecessor();
              pos_info1.laneId = pos_info1.lane->mId;
              std::cout << "20pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
              break ;
            }
          }
          pos_info1old.road = pos_info1.road;
          pos_info1.road = pos_info1.road->getPredecessor();
          pos_info1.roadId = pos_info1.road->mId;
          std::cout << "2611pos_info1.roadId " << pos_info1.roadId << std::endl;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getPredecessorLink())
          {
            pos_info1.lane = pos_info1.lane->getPredecessor();
            pos_info1.laneId = pos_info1.lane->mId;
            std::cout << "2111pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else 
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          lengthSum =pos_info1.trackS;
          (*length) =lengthSum;
          if(pos_info1old.road->getPredecessorDir() == 0)
          {
            GetRoadSuccessFlag=11;
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "2211pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.trackS;
                (*length) =lengthSum;
                return 11;
                // int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "22pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.lane = pos_info1.lane->getSuccessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "23pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
          if(pos_info1old.road->getPredecessorDir() == 1)
          {
            GetRoadSuccessFlag=10;
            // OpenDrive::LaneSection *Roadinfo1= pos_info1old.road;
            // OpenDrive::LaneSection *firstsec=(OpenDrive::LaneSection *)Roadinfo1->getFirstLaneSection();
            // pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.section = pos_info1.road->getLastLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "2411pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length) =lengthSum;
                return 11;
                // int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "24pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.lane = pos_info1.lane->getPredecessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "25pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  // int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
          nextroadI = 1;
          std::cout << "26pos_info1.roadId " << pos_info1.roadId << std::endl;
          std::cout << "27pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
          std::cout << "28pos_info1.laneId " <<pos_info1.laneId<< std::endl;          
        }      
        std::cout << "29lengthSum " << lengthSum << std::endl;
      }
      if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
      {
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.lane << std::endl;
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.lane<< std::endl;
        if (pos_info1.laneId==pos_info2.laneId)
        {
          return 11;
          int isInFrontLaneFlag = 1;
          std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
        }
        else
        {
          return 0;
        }
        endwhileFlag=1 ;
        std::cout << "32lengthSum " << lengthSum << std::endl;
      }
      else if (lengthSum >= LengthSumMax)
      {
        endwhileFlag=1;
        isInBackLaneFlag=1 ;
        std::cout << "33lengthSum " << lengthSum << std::endl;
      }
      // if (isInBackLaneFlag==1)
      // {
      //   std::cout << "34lengthSum " << lengthSum << std::endl;
      //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1, p2);
      //   if (isInFrontLaneFlag==10)
      //   {
      //     return 10;
      //   }
      //   endwhileFlag=1;
      // }
      if (endwhileFlag==1)
      {
        std::cout << "35lengthSum " << lengthSum << std::endl;
        endwhileFlag = 0;
        (*length)=lengthSum;
        lengthSum = 0;
        break;
      }
    }
    //if Successor Type is Junction
    if ((pos_info1.road->getPredecessor(1))&&(pos_info1.road->getPredecessorLink()==NULL))
    {
      // bool isInRoad = getInertialPosInfo(p1);
      std::cout << "36ODR_TYPE_JUNCTION " << std::endl;
      OpenDrive::OdrManager Manager;
      OpenDrive::Position* pos1 = Manager.createPosition();
      Manager.activatePosition(pos1);
      Manager.setLanePos(pos_info1.roadId, pos_info1.laneId, pos_info1.trackS);
      bool result = Manager.lane2inertial();
      double p1x=Manager.getInertialPos().getX();
      double p1y=Manager.getInertialPos().getY();
      double p1z=Manager.getInertialPos().getZ();
      int** RoadConnectId = getPosInfo::Preprocess(Point (p1x, p1y, p1z),heading1);
      int** JunDir= getPosInfo::GetJuncDir(Point (p1x, p1y, p1z),heading1);
      if (RoadConnectId[0][0]==-1)
      {
        return -1;
        break ;
      }
      std::cout << "yuchuliwanbi "  << std::endl;
      //read the connectRoadId/incomingId
      std::cout << "jieguochangdu "  << 2*sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0])<<std::endl;

      for (int RoadConnectI=0;RoadConnectI<(2*sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0]));RoadConnectI++)
      {
        std::cout << "nextroadid"  <<pos_info1.roadId<< std::endl;
        std::cout << "nextlaneid"  <<pos_info1.laneId<< std::endl;
        std::cout << "for  RoadConnectI"  << std::endl;
        int nextroadid=RoadConnectId[RoadConnectI][0];
        int nextlaneid=RoadConnectId[RoadConnectI][1];
        int nextRoadDir=JunDir[RoadConnectI][1];
        int IncomRoadFlag=JunDir[RoadConnectI][0];
        std::cout << "nextroadid"  <<nextroadid<< std::endl;
        std::cout << "nextlaneid"  <<nextlaneid<< std::endl;
        OpenDrive::OdrManager myManager;
        OpenDrive::Position* pos = myManager.createPosition();
        myManager.activatePosition(pos);
        myManager.setLanePos(nextroadid, nextlaneid, 0.01);
        bool result = myManager.lane2inertial();
        std::cout << "for  Connectroad1:"  <<result<< std::endl;
        OpenDrive::RoadHeader* Connectroad = myManager.getRoadHeader();
        OpenDrive::Lane* Connectlane = myManager.getLane();
        int Connectroadid=myManager.getRoadHeader()->mId;
        std::cout << "for  Connectroad"  <<Connectroadid<< std::endl;
        std::cout << " 111mConnectRoadId" <<Connectroad->mId<<std::endl;
        if (IncomRoadFlag==nextRoadDir)
        {
          std::cout << " pos_info1.roadId: " <<  std::endl;
          // pos_info1.road= Connectroad;
          // pos_info1.roadId=pos_info1.road->mId;
          // std::cout << " pos_info1.roadId: " <<  pos_info1.roadId << std::endl;

          if ((IncomRoadFlag == 10) && (nextroadI == 1))
          {
            std::cout << "6getSuccessorLink " << std::endl;

            if (pos_info1old1.road->getSuccessorDir()==0)
            {
             GetRoadSuccessFlag=11;
            }
            else
            {
             GetRoadSuccessFlag=10;
            }
            pos_info1old1.road = pos_info1old.road;
            pos_info1old.road = pos_info1.road;
            pos_info1.road = Connectroad;
            pos_info1.roadId = pos_info1.road->mId;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (Connectlane)
            {
              pos_info1.lane = Connectlane;
              pos_info1.laneId = pos_info1.lane->mId;
              std::cout << "7pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            if (( nextRoadDir== 10) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "8pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                  std::cout << "9pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length) =lengthSum;
                    return 11;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length) =lengthSum;
            }
            else if ((pos_info1.road->getPredecessorDir() == 0) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getLastLaneSection();
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getLeft())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "10pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getPredecessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
                  std::cout << "11pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                    (*length) =lengthSum;
                    return 11;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length) =lengthSum;
            }
          }
          if ((IncomRoadFlag == 11) && (nextroadI == 1))
          {
            std::cout << "12getSuccessorLink " << std::endl;
          // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
            if (pos_info1old1.road->getPredecessorDir()==0)
            {
              GetRoadSuccessFlag=11;
            }
            else
            {
              GetRoadSuccessFlag=10;
            }
            pos_info1old1.road = pos_info1old.road;
            pos_info1old.road = pos_info1.road;
            pos_info1.road = pos_info1.road->getPredecessor();
            pos_info1.roadId = pos_info1.road->mId;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (pos_info1.lane->getPredecessorLink())
            {
              pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
              std::cout << "13pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            if ((pos_info1.road->getPredecessorDir() == 1) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "14pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                  std::cout << "15pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length) =lengthSum;
                    return 11;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
            }
            lengthSum =lengthSum+pos_info1.roadlength;
            (*length) =lengthSum;
          }
          else if (nextroadI == 0)
          {
            std::cout << "118pos_info1.roadId " << pos_info1.roadId << std::endl;    
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "119pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.lane= pos_info1.lane->getPredecessor();
                pos_info1.laneId=pos_info1.lane->mId;
                std::cout << "120pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
                break ;
              }
            }
            pos_info1old.road = pos_info1.road;
            pos_info1.road = Connectroad;
            pos_info1.roadId = pos_info1.road->mId;
            std::cout << "2611pos_info1.roadId " << pos_info1.roadId << std::endl;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (Connectlane)
            {
              pos_info1.lane = Connectlane;
              pos_info1.laneId=pos_info1.lane->mId;
              std::cout << "2111pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else 
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            lengthSum =pos_info1.trackS;
            (*length) =lengthSum;
            if(IncomRoadFlag == 10)
            {
              GetRoadSuccessFlag=11;
              std::cout << "1111xiangtongfangxiang  " << std::endl;
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "2211pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30lengthSum " << lengthSum << std::endl;
                std::cout << "30pos_info2.laneId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.laneId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                 (*length) =lengthSum;
                  return 11;
                  // int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "22pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.lane = pos_info1.lane->getSuccessor();
                  pos_info1.laneId = pos_info1.lane->mId;
                  std::cout << "23pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                  std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.laneId==pos_info2.laneId)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length) =lengthSum;
                    return 11;
                    // int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length) =lengthSum;
            }
            if(IncomRoadFlag == 11)
            {
              // OpenDrive::LaneSection *Roadinfo1= pos_info1old.road;
              // OpenDrive::LaneSection *firstsec=(OpenDrive::LaneSection *)Roadinfo1->getFirstLaneSection();
              // pos_info1.section = pos_info1.road->getFirstLaneSection();
              GetRoadSuccessFlag=10;
              std::cout << "1111butongtongfangxiang  " << std::endl;
              pos_info1.section = pos_info1.road->getLastLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "2411pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length) =lengthSum;
                  return 11;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getLeft())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "24pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(lengthSum) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getPredecessorLink())
                {
                  pos_info1.lane = pos_info1.lane->getPredecessor();
                  pos_info1.laneId = pos_info1.lane->mId;
                  std::cout << "25pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                  std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.laneId==pos_info2.laneId)
                  {
                    lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                    (*length) =lengthSum;
                    return 11;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length) =lengthSum;
            }
            nextroadI = 1;
            std::cout << "26pos_info1.roadId " << pos_info1.roadId << std::endl;
            std::cout << "27pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            std::cout << "28pos_info1.laneId " <<pos_info1.laneId<< std::endl;          
          }      
          std::cout << "29lengthSum " << lengthSum << std::endl;
          if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
          {
            std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.lane << std::endl;
            std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.lane<< std::endl;
            if (pos_info1.laneId==pos_info2.laneId)
            {
              return 11;
              int isInFrontLaneFlag = 1;
              std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
            }
            else
            {
              return 0;
            }
            endwhileFlag=1 ;
            std::cout << "32lengthSum " << lengthSum << std::endl;
          }
          else if (lengthSum >= 150)
          {
            endwhileFlag=1;
            isInBackLaneFlag=1 ;
            std::cout << "33lengthSum " << lengthSum << std::endl;
          }
          // if (isInBackLaneFlag==1)
          // {
          //   std::cout << "34lengthSum " << lengthSum << std::endl;
          //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1, p2);
          //   if (isInFrontLaneFlag==11)
          //   {
          //     return 11;
          //   }
          //   endwhileFlag=1;
          // }
          if (endwhileFlag==1)
          {
            std::cout << "35lengthSum " << lengthSum << std::endl;
            endwhileFlag = 0;
            (*length)=lengthSum;
            lengthSum = 0;
            break;
          }
        }
        std::cout << "jieguo: " << nextroadid << std::endl;
        std::cout << "jieguo: " << nextlaneid << std::endl;
      }
      break;
      //need to fixed
    }
    std::cout << "isInFrontLaneFlag " << isInFrontLaneFlag << std::endl;
  }
  return 0;
}

//gx   
int getPosInfo::isInFrontLane(Point p1, Point p2,float heading1,float *length)
{
  std::cout << "isInSameLane enable " << std::endl;
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
    return 0;
  posInfo pos_info1 = m_posInfo;
  isInRoad = getInertialPosInfo(p2);
  posInfo pos_info2 = m_posInfo;
  if (!isInRoad)
    return 0;
  int isInFrontLaneFlag = 0;
  int isInBackLaneFlag = 0;
  int LengthSumMax=150;
  double lengthSum=0;



  //check if in same road
  if (pos_info1.roadId == pos_info2.roadId)
  {
    std::cout << "isInSameRoad " << std::endl;
    // check if in same laneSection
    if (pos_info1.sectionS == pos_info2.sectionS)
    {
      std::cout << "isInSameSec " << std::endl;
      //the same road and the same lane
      if (pos_info1.laneId == pos_info2.laneId)
      {
        std::cout << "isInSamelane " << std::endl;
        if (pos_info1.trackS < pos_info2.trackS)
        {
          lengthSum=pos_info2.trackS-pos_info1.trackS ;
          (*length)=lengthSum;
          int isInFrontLaneFlag = 1;
          std::cout << "in same lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
          return 10;
        }
      }
    }
    else if (pos_info1.sectionS < pos_info2.sectionS)
    {
      bool endwhileFlag = 0;
      while ((pos_info1.roadId == pos_info2.roadId) && (pos_info1.sectionS < pos_info2.sectionS))
      {
        std::cout << "isInfrontsec " << std::endl;
        // pos_info1.sectionS=pos_info1.sectionS->getNextSections();
        OpenDrive::LaneSection *secInfo1 = pos_info1.section;
        OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
        pos_info1.section = nextSecInfo1;
        // OpenDrive::LaneLink* laneInfo1=pos_info1.lane->getSuccessorLink();
        // pos_info1.laneId = laneInfo1-> mLaneId;
        pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
        std::cout << "pos_info1.laneId " << pos_info1.laneId << std::endl;
        std::cout << "pos_info2.laneId " << pos_info2.laneId << std::endl;
        if (pos_info1.section == pos_info2.section)
        {
          if (pos_info1.laneId == pos_info2.laneId)
          {
            lengthSum=pos_info2.trackS-pos_info1.trackS ;
            (*length)=lengthSum;
            return 10;
            //2 is at the front of 1;
            int isInFrontLaneFlag = 1;
            std::cout << "in same road not one section isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
          }
          else
          {
            return -1;
            //the same road but not the same lane
          }
          endwhileFlag = 1;
        }
        if (endwhileFlag == 1)
        {
          endwhileFlag = 0;
          break;
        }
      }
    }
  }
  // check if not in same road ,if in the front road;
  bool endwhileFlag = 0;
  bool nextroadI = 0;
  posInfo pos_info1old = pos_info1;
  posInfo pos_info1old1 = pos_info1;
  pos_info1.roadlength = 0;
  int GetRoadSuccessFlag=11;

  // pos_info1old.road = pos_info1.road;
  while ((pos_info1.roadId != pos_info2.roadId) && (lengthSum < LengthSumMax))
  {
    // std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
    // pos_info1.roadlength = pos_info1.road->mLength;
    std::cout << "isfrontroad " << std::endl;
    std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
    std::cout << "2roadlength " << pos_info1.roadlength << std::endl;
    if (((pos_info1.road->getSuccessorLink()!=NULL)&& GetRoadSuccessFlag==11)
        ||((pos_info1.road->getPredecessorLink()!=NULL)&& GetRoadSuccessFlag==10))
    {
      std::cout << "3getSuccessorLink " << pos_info1.roadId<<std::endl;
      std::cout << "4pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
      std::cout << "5pos_info1.lane  " <<pos_info1.laneId << std::endl;
      std::cout << "77GetRoadSuccessFlag :" <<GetRoadSuccessFlag << std::endl;
      bool nextRoadFlag = 0;
      //if have next sec
      {
        // if ((pos_info1old1.road->getSuccessorDir() == 0) && (nextroadI == 1))
        if ((GetRoadSuccessFlag==11) && (nextroadI == 1))
        {
          std::cout << "6getSuccessorLink " << std::endl;
          pos_info1old1.road = pos_info1old.road;
          pos_info1old.road = pos_info1.road;
          if (pos_info1old1.road->getSuccessorDir()==0)
          {
            GetRoadSuccessFlag=11;
          }
          else
          {
            GetRoadSuccessFlag=10;
          }
          pos_info1.roadId = pos_info1.road->mId;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getSuccessorLink())
          {
            pos_info1.lane = pos_info1.lane->getSuccessor();
            pos_info1.laneId = pos_info1.lane->mId;
            std::cout << "7pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          if ((pos_info1old.road->getSuccessorDir() == 0) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "8pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.lane = pos_info1.lane->getSuccessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "9pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.road->mLength;
            (*length)=lengthSum;
          }
          else if ((pos_info1old.road->getSuccessorDir() == 1) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getLastLaneSection();
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "10pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.lane = pos_info1.lane->getPredecessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "11pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.road->mLength;
            (*length)=lengthSum;
          }
        }

        // if ((pos_info1old1.road->getSuccessorDir() == 1) && (nextroadI == 1))
        if ((GetRoadSuccessFlag==10) && (nextroadI == 1))
        {
          std::cout << "12getSuccessorLink " << std::endl;
        // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
          pos_info1old1.road = pos_info1old.road;
          pos_info1old.road = pos_info1.road;
          if (pos_info1old1.road->getPredecessorDir()==0)
          {
            GetRoadSuccessFlag=11;
          }
          else
          {
            GetRoadSuccessFlag=10;
          }
          pos_info1.road = pos_info1.road->getPredecessor();
          pos_info1.roadId = pos_info1.road->mId;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getPredecessorLink())
          {
            pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
            std::cout << "13pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          if ((pos_info1old.road->getSuccessorDir() == 0) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "14pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                std::cout << "15pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.lane==pos_info2.lane)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
          }
          else if ((pos_info1old.road->getSuccessorDir() == 1) && (nextroadI == 1))
          {
            pos_info1.section = pos_info1.road->getLastLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "16pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
                std::cout << "17pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                if (pos_info1.lane==pos_info2.lane)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
          }
        }
        else if (nextroadI == 0)
        {
          std::cout << "18pos_info1.roadId " << pos_info1.roadId << std::endl;    
          while (pos_info1.section->getRight())
          {           
            OpenDrive::LaneSection *secInfo1 = pos_info1.section;
            OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
            pos_info1.section = nextSecInfo1;
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "19pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            // pos_info1.sectionS = nextSecInfo1->mS;
            if (pos_info1.lane->getSuccessorLink())
            {
              pos_info1.lane = pos_info1.lane->getSuccessor();
              pos_info1.laneId = pos_info1.lane->mId;
              std::cout << "20pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
              break ;
            }
          }
          pos_info1old.road = pos_info1.road;
          pos_info1.road = pos_info1.road->getSuccessor();
          pos_info1.roadId = pos_info1.road->mId;
          std::cout << "2611pos_info1.roadId " << pos_info1.roadId << std::endl;
          pos_info1.roadlength = pos_info1.road->mLength;
          if (pos_info1.lane->getSuccessorLink())
          {
            pos_info1.lane = pos_info1.lane->getSuccessor();
            pos_info1.laneId = pos_info1.lane->mId;
            std::cout << "2111pos_info1.laneId " <<pos_info1.laneId<< std::endl;
          }
          else 
          {
            endwhileFlag=1 ;
            isInBackLaneFlag=1 ;
          }
          lengthSum =pos_info1.roadlength -pos_info1.trackS;
          std::cout << "lengthSum " <<lengthSum<< std::endl;
          if(pos_info1old.road->getSuccessorDir() == 0)
          {
            GetRoadSuccessFlag=11;
            pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "2211pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30lengthSum " << lengthSum << std::endl;
              std::cout << "30pos_info2.laneId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.laneId " << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum=lengthSum+pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "22pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              // lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.lane = pos_info1.lane->getSuccessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "23pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30lengthSum " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.road->mLength;
          }
          if(pos_info1old.road->getSuccessorDir() == 1)
          {
            GetRoadSuccessFlag=10;
            // OpenDrive::LaneSection *Roadinfo1= pos_info1old.road;
            // OpenDrive::LaneSection *firstsec=(OpenDrive::LaneSection *)Roadinfo1->getFirstLaneSection();
            // pos_info1.section = pos_info1.road->getFirstLaneSection();
            pos_info1.section = pos_info1.road->getLastLaneSection();
            pos_info1.sectionS = pos_info1.section->mS;
            std::cout << "2411pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
            {
              std::cout << "30lengthSum " << lengthSum << std::endl;
              std::cout << "30pos_info2.laneId " << pos_info2.laneId << std::endl;
              std::cout << "30pos_info1.laneId" << pos_info1.laneId<< std::endl;
              if (pos_info1.laneId==pos_info2.laneId)
              {
                lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                (*length)=lengthSum;
                return 10;
                int isInFrontLaneFlag = 1;
                std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
              }
              else
              {
                return 0;
              }
              endwhileFlag=1 ;
              std::cout << "32lengthSum " << lengthSum << std::endl;
            }
            while (pos_info1.section->getLeft())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "24pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getPredecessorLink())
              {
                pos_info1.lane = pos_info1.lane->getPredecessor();
                pos_info1.laneId = pos_info1.lane->mId;
                std::cout << "25pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else 
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
              }
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
               std::cout << "30lengthSum " << lengthSum << std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
                break ;
              }
            }
            lengthSum =lengthSum+pos_info1.road->mLength;
          }
          nextroadI = 1;
          std::cout << "7777GetRoadSuccessFlag :" <<GetRoadSuccessFlag << std::endl;
          std::cout << "26pos_info1.roadId " << pos_info1.roadId << std::endl;
          std::cout << "27pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
          std::cout << "28pos_info1.laneId " <<pos_info1.laneId<< std::endl;          
        }      
        std::cout << "29lengthSum " << lengthSum << std::endl;
      }
      if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
      {
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.lane << std::endl;
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.lane<< std::endl;
        if (pos_info1.laneId==pos_info2.laneId)
        {
          return 10;
          int isInFrontLaneFlag = 1;
          std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
        }
        else
        {
          return 0;
        }
        endwhileFlag=1 ;
        std::cout << "32lengthSum " << lengthSum << std::endl;
      }
      else if (lengthSum >= LengthSumMax)
      {
        endwhileFlag=1;
        // isInBackLaneFlag=1 ;
        std::cout << "33lengthSum " << lengthSum << std::endl;
      }
      if (isInBackLaneFlag==1)
      {
      //   std::cout << "34lengthSum " << lengthSum << std::endl;
      //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1, p2,heading1);
      //   if (isInFrontLaneFlag==11)
      //   {
      //     return 11;
      //   }
        endwhileFlag=1;
      }
      if (endwhileFlag==1)
      {
        std::cout << "35lengthSum " << lengthSum << std::endl;
        // endwhileFlag = 0;
        // (*length )= lengthSum;
        lengthSum = 0;
        break;
      }
    }
    //if Successor Type is Junction
    if (((pos_info1.road->getSuccessor(1))&&(pos_info1.road->getSuccessorLink()==NULL)&&GetRoadSuccessFlag==11)
        ||((pos_info1.road->getSuccessor(1))&&(pos_info1.road->getSuccessorLink()==NULL)&&GetRoadSuccessFlag==10))
    {
      std::cout << "36ODR_TYPE_JUNCTION " << std::endl;
      // bool isInRoad = getInertialPosInfo(p1);
      OpenDrive::OdrManager Manager;
      OpenDrive::Position* pos1 = Manager.createPosition();
      Manager.activatePosition(pos1);
      Manager.setLanePos(pos_info1.roadId, pos_info1.laneId, pos_info1.trackS);
      bool result = Manager.lane2inertial();
      double p1x=Manager.getInertialPos().getX();
      double p1y=Manager.getInertialPos().getY();
      double p1z=Manager.getInertialPos().getZ();
      int** RoadConnectId = getPosInfo::Preprocess(Point (p1x, p1y, p1z),heading1);
      int** JunDir= getPosInfo::GetJuncDir(Point (p1x, p1y, p1z),heading1);
      if (RoadConnectId[0][0]==-1)
      {
        return -1;
        break ;
      }
      std::cout << "yuchuliwanbi "  << std::endl;
      //read the connectRoadId/incomingId
      std::cout << "jieguochangdu "  << 2*sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0])<<std::endl;
      for (int RoadConnectI=0;RoadConnectI<(2*sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0]));RoadConnectI++)
      {
        // std::cout << "nextroadid"  <<pos_info1.roadId<< std::endl;
        // std::cout << "nextlaneid"  <<pos_info1.laneId<< std::endl;
        // std::cout << "for  RoadConnectI"  << std::endl;
        int nextroadid=RoadConnectId[RoadConnectI][0];
        int nextlaneid=RoadConnectId[RoadConnectI][1];
        int nextRoadDir=JunDir[RoadConnectI][1];
        int IncomRoadFlag=JunDir[RoadConnectI][0];
        // std::cout << "nextroadid"  <<nextroadid<< std::endl;
        // std::cout << "nextlaneid"  <<nextlaneid<< std::endl;
        OpenDrive::OdrManager myManager;
        OpenDrive::Position* pos = myManager.createPosition();
        myManager.activatePosition(pos);
        myManager.setLanePos(nextroadid, nextlaneid, 0.01);
        bool result = myManager.lane2inertial();
        std::cout << "for  Connectroad1:"  <<result<< std::endl;
        OpenDrive::RoadHeader* Connectroad = myManager.getRoadHeader();
        OpenDrive::Lane* Connectlane = myManager.getLane();
        int Connectroadid=myManager.getRoadHeader()->mId;
        std::cout << "for  Connectroad"  <<Connectroadid<< std::endl;
        // std::cout << " 111mConnectRoadId" <<Connectroad->mId<<std::endl;
        //pos_info1.road为Incoming&contactpoint为0，或者pos_info1.road为Connecting&contactpoint为1
        if (IncomRoadFlag==nextRoadDir)
        {
          // std::cout << " pos_info1.roadId: " <<  std::endl;
          // pos_info1.road= Connectroad;
          // pos_info1.roadId=pos_info1.road->mId;
          // std::cout << " pos_info1.roadId: " <<  pos_info1.roadId << std::endl;
          std::cout << "6getSuccessorLink " << std::endl;
         // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();

          if ((IncomRoadFlag == 10) && (nextroadI == 1))
          {
            std::cout << "6getSuccessorLink " << std::endl;
          // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
            pos_info1old1.road = pos_info1old.road;
            pos_info1old.road = pos_info1.road;
            pos_info1.road = Connectroad;
            pos_info1.roadId = pos_info1.road->mId;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (pos_info1old1.road->getSuccessorDir()==0)
            {
             GetRoadSuccessFlag=11;
            }
            else
            {
             GetRoadSuccessFlag=10;
            }
            if (Connectlane)
            {
              pos_info1.lane = Connectlane;
              pos_info1.laneId = pos_info1.lane->mId;
              std::cout << "7pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            if (( nextRoadDir== 10) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "8pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                  std::cout << "9pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length)=lengthSum;
                    return 10;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length)=lengthSum;
            }
            else if ((pos_info1.road->getPredecessorDir() == 0) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getLastLaneSection();
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getLeft())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "10pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getPredecessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
                  std::cout << "11pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                    (*length)=lengthSum;
                    return 10;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length)=lengthSum;
            }
          }
          if ((IncomRoadFlag == 11) && (nextroadI == 1))
          {
            std::cout << "12getPredecessorLink " << std::endl;
          // OpenDrive::LaneLink* laneInfo2=pos_info2.lane->getSuccessorLink();
            pos_info1old1.road = pos_info1old.road;
            pos_info1old.road = pos_info1.road;
            pos_info1.road = pos_info1.road->getPredecessor();
            pos_info1.roadId = pos_info1.road->mId;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (pos_info1.lane->getPredecessorLink())
            {
              pos_info1.laneId = pos_info1.lane->getPredecessor()->mId;
              std::cout << "13pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            if ((pos_info1.road->getPredecessorDir() == 1) && (nextroadI == 1))
            {
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "88sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "14pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                  std::cout << "15pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.lane==pos_info2.lane)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length)=lengthSum;
                    return 10;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length)=lengthSum;
            }
          }
          else if (nextroadI == 0)
          {
            // std::cout << "118pos_info1.roadId " << pos_info1.roadId << std::endl;    
            while (pos_info1.section->getRight())
            {           
              OpenDrive::LaneSection *secInfo1 = pos_info1.section;
              OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
              pos_info1.section = nextSecInfo1;
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "119pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
              // pos_info1.sectionS = nextSecInfo1->mS;
              if (pos_info1.lane->getSuccessorLink())
              {
                pos_info1.lane= pos_info1.lane->getSuccessor();
                pos_info1.laneId=pos_info1.lane->mId;
                std::cout << "120pos_info1.laneId " <<pos_info1.laneId<< std::endl;
              }
              else
              {
                endwhileFlag=1 ;
                isInBackLaneFlag=1 ;
                break ;
              }
            }
            pos_info1old.road = pos_info1.road;
            pos_info1.road = Connectroad;
            pos_info1.roadId = pos_info1.road->mId;
            std::cout << "2611pos_info1.roadId " << pos_info1.roadId << std::endl;
            pos_info1.roadlength = pos_info1.road->mLength;
            if (Connectlane)
            {
              pos_info1.lane = Connectlane;
              pos_info1.laneId=pos_info1.lane->mId;
              std::cout << "2111pos_info1.laneId " <<pos_info1.laneId<< std::endl;
            }
            else 
            {
              endwhileFlag=1 ;
              isInBackLaneFlag=1 ;
            }
            lengthSum =pos_info1.roadlength -pos_info1.trackS;
            (*length)=lengthSum;
            if(IncomRoadFlag == 10)
            {
              GetRoadSuccessFlag=11;
              std::cout << "1111xiangtongfangxiang  " << std::endl;
              pos_info1.section = pos_info1.road->getFirstLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              // std::cout << "2211pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                // std::cout << "30lengthSum " << lengthSum << std::endl;
                std::cout << "30pos_info2.laneId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.laneId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getRight())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "22pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getSuccessorLink())
                {
                  pos_info1.lane = pos_info1.lane->getSuccessor();
                  pos_info1.laneId = pos_info1.lane->mId;
                  std::cout << "23pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                  std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.laneId==pos_info2.laneId)
                  {
                    lengthSum =lengthSum+pos_info2.trackS;
                    (*length)=lengthSum;
                    return 10;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length)=lengthSum;
            }
            if(IncomRoadFlag == 11)
            {
              GetRoadSuccessFlag=10;
              // OpenDrive::LaneSection *Roadinfo1= pos_info1old.road;
              // OpenDrive::LaneSection *firstsec=(OpenDrive::LaneSection *)Roadinfo1->getFirstLaneSection();
              // pos_info1.section = pos_info1.road->getFirstLaneSection();
              std::cout << "1111butongtongfangxiang  " << std::endl;
              pos_info1.section = pos_info1.road->getLastLaneSection();
              pos_info1.sectionS = pos_info1.section->mS;
              std::cout << "2411pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
              if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
              {
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.laneId << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.laneId<< std::endl;
                if (pos_info1.laneId==pos_info2.laneId)
                {
                  lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                  (*length)=lengthSum;
                  return 10;
                  int isInFrontLaneFlag = 1;
                  std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                }
                else
                {
                  return 0;
                }
                endwhileFlag=1 ;
                std::cout << "32lengthSum " << lengthSum << std::endl;
              }
              while (pos_info1.section->getLeft())
              {           
                OpenDrive::LaneSection *secInfo1 = pos_info1.section;
                OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
                pos_info1.section = nextSecInfo1;
                pos_info1.sectionS = pos_info1.section->mS;
                std::cout << "24pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
                lengthSum = fabs(pos_info1.trackS) + fabs(pos_info1.section->mS);
                // pos_info1.sectionS = nextSecInfo1->mS;
                if (pos_info1.lane->getPredecessorLink())
                {
                  pos_info1.lane = pos_info1.lane->getPredecessor();
                  pos_info1.laneId = pos_info1.lane->mId;
                  std::cout << "25pos_info1.laneId " <<pos_info1.laneId<< std::endl;
                }
                else 
                {
                  endwhileFlag=1 ;
                  isInBackLaneFlag=1 ;
                }
                if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
                {
                  std::cout << "30pos_info1.roadId == pos_info2.roadId " << lengthSum << std::endl;
                  if (pos_info1.laneId==pos_info2.laneId)
                  {
                    lengthSum =lengthSum+pos_info2.roadlength-pos_info2.trackS;
                    (*length)=lengthSum;
                    return 10;
                    int isInFrontLaneFlag = 1;
                    std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
                  }
                  else
                  {
                    return 0;
                  }
                  endwhileFlag=1 ;
                  std::cout << "32lengthSum " << lengthSum << std::endl;
                  break ;
                }
              }
              lengthSum =lengthSum+pos_info1.roadlength;
              (*length)=lengthSum;
            }
            nextroadI = 1;
            std::cout << "26pos_info1.roadId " << pos_info1.roadId << std::endl;
            std::cout << "27pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
            std::cout << "28pos_info1.laneId " <<pos_info1.laneId<< std::endl;          
          }  

          std::cout << "29lengthSum " << lengthSum << std::endl;
          if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
          {
            std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.lane << std::endl;
            std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.lane<< std::endl;
            if (pos_info1.laneId==pos_info2.laneId)
            {
              return 10;
              int isInFrontLaneFlag = 1;
              std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
            }
            else
            {
              return 0;
            }
            endwhileFlag=1 ;
            std::cout << "32lengthSum " << lengthSum << std::endl;
          }
          else if (lengthSum >= LengthSumMax)
          {
            endwhileFlag=1;
            // isInBackLaneFlag=1 ;
            std::cout << "33lengthSum " << lengthSum << std::endl;
          }
          // if (isInBackLaneFlag==1)
          // {
          //   std::cout << "34lengthSum " << lengthSum << std::endl;
          //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1, p2,heading1);
          //   if (isInFrontLaneFlag==11)
          //   {
          //     return 11;
          //   }
          //   endwhileFlag=1;
          // }
          if (endwhileFlag==1)
          {
            std::cout << "35lengthSum " << lengthSum << std::endl;
            // endwhileFlag = 0;
            // (*length) =lengthSum;
            lengthSum = 0;
            break;
          }
        }
        if (RoadConnectI>=(2*sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0])))
        {
          break;
        }
        std::cout << "jieguo: " << nextroadid << std::endl;
        std::cout << "jieguo: " << nextlaneid << std::endl;
        std::cout << "26pos_info1.roadId " << pos_info1.roadId << std::endl;
        std::cout << "27pos_info1.sectionS  " <<pos_info1.sectionS << std::endl;
        std::cout << "28pos_info1.laneId " <<pos_info1.laneId<< std::endl;  
      }
      //need to fixed
      if ((pos_info1.roadId == pos_info2.roadId)&&(pos_info1.section == pos_info2.section))
      {
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info2.lane << std::endl;
        std::cout << "30pos_info1.roadId == pos_info2.roadId " << pos_info1.lane<< std::endl;
        if (pos_info1.laneId==pos_info2.laneId)
        {
          return 10;
          int isInFrontLaneFlag = 1;
          std::cout << "31in not one road not one lane isInFrontLaneFlag :" << isInFrontLaneFlag << endl;
        }
        else
        {
          return 0;
        }
        endwhileFlag=1 ;
        std::cout << "32lengthSum " << lengthSum << std::endl;
      }
      else if (lengthSum >= LengthSumMax)
      {
        endwhileFlag=1;
        isInBackLaneFlag=1 ;
        std::cout << "33lengthSum " << lengthSum << std::endl;
      }
      if (isInBackLaneFlag==1)
      {
        // std::cout << "34lengthSum " << lengthSum << std::endl;
        // int isInFrontLaneFlag = getPosInfo::isInBackLaneNew(p1, p2,heading1);
        // if (isInFrontLaneFlag==11)
        // {
        //   return 11;
        // }
        endwhileFlag=1;
      }
      if (endwhileFlag==1)
      {
        std::cout << "35lengthSum " << lengthSum << std::endl;
        endwhileFlag = 0;
        (*length)=lengthSum;
        lengthSum = 0;
        break;
      }
    }
    std::cout << "isInFrontLaneFlag " << isInFrontLaneFlag << std::endl;
  }
  return 0;
}

//gx
bool getPosInfo::isFrontCar(Point p1, Point p2, float heading1, float heading2,float *length)
{
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
    return 0;
  isInRoad = getInertialPosInfo(p2);
  if (!isInRoad)
    return 0;
  std::cout << "is front car enable " << std::endl;
  float pi=4*atan(1);
  // std::cout << "isInFrontOrBackLaneFlag " << isInFrontOrBackLaneFlag << std::endl;
  // std::cout << "heading1 " << heading1 << std::endl;
  if ((((pi/2) > heading1 )&&(heading1> 0))||(((-pi/2) < heading1)&&(heading1 < 0)))
  {
    int isInFrontOrBackLaneFlag = getPosInfo::isInFrontLane(p1, p2, heading1, length);
    std::cout << "1isInFrontOrBackLaneFlag:" << isInFrontOrBackLaneFlag<<std::endl;
    if (isInFrontOrBackLaneFlag == 10)
    {
      std::cout << "isFrontCar " << std::endl;
      return 1;
    }
  }
  else if ((((pi/2) <= heading1) && (heading1 <= pi)) || (((-pi/2)  >= heading1) && (heading1 >= (-pi))))
  {
    int isInFrontOrBackLaneFlag = getPosInfo::isInBackLane(p1, p2, heading1,length);
    std::cout << "2isInFrontOrBackLaneFlag:" << isInFrontOrBackLaneFlag<<std::endl;
    if (isInFrontOrBackLaneFlag == 10)
    {
      std::cout << "isFrontCar " << std::endl;
      return 1;
    }
  }
  std::cout << "is front car end " << std::endl;
  return 0;
}

//gx
bool getPosInfo::isBackCar(Point p1, Point p2, float heading1, float heading2,float *length)
{
  bool isInRoad = getInertialPosInfo(p1);
  if (!isInRoad)
    return 0;
  isInRoad = getInertialPosInfo(p2);
  if (!isInRoad)
    return 0;
  float pi=4*atan(1);
  if (((pi/2) > heading1 > 0) || ((-pi/2) < heading1 < 0))
  {
    int isInFrontOrBackLaneFlag = getPosInfo::isInBackLane(p1, p2,heading1,length);
    if (isInFrontOrBackLaneFlag == 11)
    {
      std::cout << "isBackCar " << std::endl;
      return 1;
    }
  }
  else if ((((pi/2) <= heading1) && (heading1 <= pi)) || (((-pi/2)  >= heading1) && (heading1 >= (-pi))))
  {
    int isInFrontOrBackLaneFlag = getPosInfo::isInFrontLane(p1, p2,heading1,length);
    if (isInFrontOrBackLaneFlag == 11)
    {
      std::cout << "isBackCar " << std::endl;
      return 1;
    }
  }
  return 0;
}

//jm/hux
bool getPosInfo::isOnRoadMark(const Point& p) {
  OpenDrive::Position* pos = m_manager.createPosition();
  m_manager.activatePosition(pos);

  m_manager.setInertialPos(p.x, p.y, p.z);
  bool result = m_manager.inertial2lane();

  // TODO:it's better to get the width of roadmark from opendrive file or vtd
  const double wheelWidth = 0.2;

  if (result) {
    OpenDrive::Lane* myLane = m_manager.getLane();
    OpenDrive::RoadHeader* header = m_manager.getRoadHeader();
    double laneWidth = m_manager.getLaneWidth();
    OpenDrive::LaneCoord posLane = m_manager.getLanePos();
    double offsetFromLaneCenter = posLane.getOffset();
    //cout <<"laneOffset is:" <<offsetFromLaneCenter<< endl;

    double dist = laneWidth / 2 - fabs(offsetFromLaneCenter) - wheelWidth/2;


      if(myLane->mId > 0)           // lane is at left of the road
      {
        if(offsetFromLaneCenter >= 0 )  // position is at left of the lane, then myLane->roadMark is the target roadMark
        {
          OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
          
          if(myRoadMark == NULL)   
          {
            return false;
          }
          cout<<"roadMarkType:"<< myRoadMark->getType() <<endl;


          if (dist < myRoadMark->getWidth() / 2)
          {

            if(myRoadMark->getType() == 1 || myRoadMark->getType() == 3)
            {
              cout << "ShiXian!" << endl;
              return true;
            }
            else
            {
              cout << "XuXian!" << endl;
              return false;  
            }
          }
          else
          {
            cout << "too far from roadMark" << endl;
            return false;
          }  
        }   
        else        // position is at right of the lane, then myRIGHTLane->roadMark is the target roadMark
        {
          OpenDrive::Lane* myRightLane = (OpenDrive::Lane*)myLane->getRight(); 

          if(myRightLane == NULL)
            return false;
          OpenDrive::RoadMark* myRoadMark = myRightLane->getFirstRoadMark();
          cout<<"roadMarkType:"<< myRoadMark->getType() <<endl;
          if(myRoadMark == NULL)   
          {
            return false;
          }
          if (dist < myRoadMark->getWidth() / 2)
          {
            if(myRoadMark->getType() == 1 || myRoadMark->getType() == 3)
            {
              cout << "ShiXian!" << endl;
              return true;
            }
            else
            {
              cout << "XuXian!" << endl;
              return false;  
            }
          }
          else
          {
            cout << "too far from roadMark" << endl;
            return false;
          } 
        }  
      }
      else           // lane is at right of the road
      {
        if(offsetFromLaneCenter >= 0 )  // position is at left of the lane, then myLEFTLane->roadMark is the target roadMark
        {
          OpenDrive::Lane* myLeftLane = (OpenDrive::Lane*)myLane->getLeft(); 
          if(myLeftLane == NULL)
            return false;
          OpenDrive::RoadMark* myRoadMark = myLeftLane->getFirstRoadMark();
          cout<<"roadMarkType:"<< myRoadMark->getType() <<endl;
          if(myRoadMark == NULL)   
          {
            return false;
          }
          if (dist < myRoadMark->getWidth() / 2)
          {
            if(myRoadMark->getType() == 1 || myRoadMark->getType() == 3)
            {
              cout << "ShiXian!" << endl;
              return true;
            }
            else
            {
              cout << "XuXian!" << endl;
              return false;  
            }
          }
          else
          {
            cout << "too far from roadMark" << endl;
            return false;
          } 
        }   
        else        // position is at right of the lane, then myLane->roadMark is the target roadMark
        {
          OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
          cout<<"roadMarkType:"<< myRoadMark->getType() <<endl;
          if(myRoadMark == NULL)   
          {
            return false;
          }
          if (dist < myRoadMark->getWidth() / 2)
          {
            if(myRoadMark->getType() == 1 || myRoadMark->getType() == 3)
            {
              cout << "ShiXian!" << endl;
              return true;
            }
            else
            {
              cout << "XuXian!" << endl;
              return false;  
            }
          } 
          else
          {
            cout << "too far from roadMark" << endl;
            return false;
          } 
        }  
      }      




      // if(offsetFromLaneCenter >= 0 )  // position is at left of the lane
      // {
      //   if(myLane->mId > 0)           // lane is at left of the road, then myLane->roadMark is the target roadMark
      //   {
      //     OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
      //     if(myRoadMark->getType() == 1)
      //       return true;
      //     else
      //       return false;          
      //   }
      //   else              // lane is at right of the road, then myLeftLane->roadMark is the target roadMark
      //   {
      //     OpenDrive::Lane* myLeftlane = (OpenDrive::Lane*)myLane->getLeft();        
      //     OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
      //     if(myRoadMark->getType() == 1)
      //       return true;
      //     else
      //       return false;           
      //   }
      // }
      // else                               // position is at left of the lane
      // {

      // }

    //cout <<"myLaneId is:" <<myLane->mId<< endl;
    //OpenDrive::Lane* myLeftlane = (OpenDrive::Lane*)myLane->getLeft();
    //cout <<"myLeftlaneId is:" << myLeftlane->mId<< endl;

  }
  return result;
}

//jm
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

//gx
bool getPosInfo::isSameDirectionWithRoad(Point p,float heading1)
{
  getInertialPosInfo(p);
  float pi=atan(1)*4;
  float CarHeading=m_posInfo.roadheading-heading1 ;
  // std::cout<<"m_posInfo.roadheading"<<m_posInfo.roadheading<<std::endl;
  if (((pi/2) > CarHeading > 0) || ((-pi/2) < CarHeading < 0))
  {
    return 1;
  }
  return 0;
}


//wh
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

//wh
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

//wh
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

//wh
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

//is Turning:gx
bool getPosInfo::isInSce2_46_7(Point p)
{
  bool isInRoad = getInertialPosInfo(p);
  double pi=atan(1)*4;
  double headingChangeMin = (pi/36);
  // if (m_manager.getRoadHeader()->getJunctionNo()==-1)
  // {
  //   return 0 ;
  // }
  // else
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
      //
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

// TODO isTheSteepSlope:gx
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

// TODO isTopRam:gx
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
  // OpenDrive::Elevation*  lastElevation = (OpenDrive::Elevation* )myElevation->getLeft();
  // std::cout << "isTopRam start11111." << std::endl;
  // if(m_posInfo.ElvmB != (lastElevation->mB))
  // {
  //   float sChangeLast= (lastElevation->mSEnd-lastElevation->mS)/10;
  //   float sChangeL=sChangeLast*9;
  //   float hChangeLast=lastElevation->mB*sChangeL+lastElevation->mC*sChangeL*sChangeL+lastElevation->mD*sChangeL*sChangeL*sChangeL;
  //   float sChange= (m_posInfo.ElvmSEnd-m_posInfo.ElvmS)/10;
  //   float hChange=m_posInfo.ElvmB*sChange+m_posInfo.ElvmC*sChange*sChange+m_posInfo.ElvmD*sChange*sChange*sChange;
  //   if (hChangeLast>0)
  //   {
  //     return 1;
  //   }
  //   else if (hChangeLast==0)
  //   {
  //     if (hChange<0)
  //     {
  //       return 1;
  //     }
  //   }
  //   else
  //   {
  //     if (hChange<=0)
  //     {
  //       return 1;
  //     }
  //   }
  // }
}

//wh
bool getPosInfo::isInSce2_46_3(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    double curv = m_manager.getCurvature();
    if (curv > 0.02 || curv < -0.02) {
        return true;
    }
    return false;
}

//TODO isNarrowRoad:gx
bool getPosInfo::isInSec2_46_4(Point p)
{
  std::cout << "isInSec2_46_4 start"<<std::endl;
  bool isInRoad = getInertialPosInfo(p);
  if (isInRoad)
  {
    double laneWidth = m_manager.getLaneWidth();
    double RoadWidth;
    double RoadWidthMax=5;
    bool endwhileFlag=0;
    posInfo posInfo1=m_posInfo;
    posInfo posInfo2=m_posInfo;
    OpenDrive::Lane *laneInfonext = posInfo1.lane;
    OpenDrive::Lane *laneInfolast = posInfo2.lane;
    RoadWidth=posInfo1.laneWidth;
    std::cout << "lanewidth"<< RoadWidth<<std::endl;
    while(((OpenDrive::Lane *)laneInfonext->getRight()))
    {
      OpenDrive::Lane *nextLaneInfo1 = (OpenDrive::Lane *)laneInfonext->getRight();
      posInfo1.lane = nextLaneInfo1;
      posInfo1.laneId= posInfo1.lane->mId;
      laneInfonext= nextLaneInfo1;
      std::cout << "getRight:"<< std::endl;
      if (laneInfonext->mType==((OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_TAXI)))
      {
        m_manager.setLanePos(posInfo1.roadId,posInfo1.laneId , posInfo1.trackS);
        m_manager.lane2inertial();
        posInfo1.laneWidth=m_manager.getLaneWidth();
        RoadWidth=RoadWidth+posInfo1.laneWidth;
        std::cout << "laneId:"<< posInfo1.laneId<<std::endl;
        std::cout << "LaneWidth:"<<posInfo1.laneWidth<<std::endl;
        std::cout << "RoadWidth:"<< RoadWidth<<std::endl;
      }
      if ((RoadWidth>RoadWidthMax))
      {
        return 0 ;
        break ;
      }
      else if((OpenDrive::Lane *)laneInfonext->getRight()==NULL)
      {
        endwhileFlag=1;
        break;
      }
    }
    if (endwhileFlag==1)
    {
      while((OpenDrive::Lane *)laneInfolast->getLeft())
      {
        OpenDrive::Lane *lastLaneInfo1 = (OpenDrive::Lane *)laneInfolast->getLeft();
        m_posInfo.lane = lastLaneInfo1;
        laneInfolast=lastLaneInfo1;
        if (laneInfolast->mType==(OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_TAXI))
        {
          m_manager.setLanePos(posInfo2.roadId,posInfo2.laneId , posInfo2.trackS);
          m_manager.lane2inertial();
          posInfo2.laneWidth=m_manager.getLaneWidth();
          RoadWidth=RoadWidth+posInfo2.laneWidth;
          std::cout << "getLeft:"<< std::endl;
          std::cout << "laneId:"<< m_posInfo.laneId<<std::endl;
          std::cout << "LaneWidth:"<<posInfo2.laneWidth<<std::endl;
          std::cout << "RoadWidth:"<< RoadWidth<<std::endl;
        }
        if (RoadWidth>RoadWidthMax)
        {
          return 0 ;
          break ;
        }
        if ((OpenDrive::Lane *)laneInfolast->getLeft()==NULL)
        {
          break ;
        }
      }
    }
    if (RoadWidth<=RoadWidthMax) 
    {
      return 1 ;
    }
  }
  return 0 ;
}

//TODO isNarrowBridge:gx
//bridge can not teat
bool getPosInfo::isInSec2_46_5(Point p)
{
  std::cout << "isInSec2_46_5 start"<<std::endl;
  bool isInRoad = getInertialPosInfo(p);
  bool isOnBridgeFlag=m_manager.onBridge();
  // OpenDrive::Bridge* isOnBridgeFlag=m_posInfo.road->getFirstBridge();
  // int isOnBridgeid=m_posInfo.road->getFirstBridge()->mId;
  // std::cout << "isOnBridge:"<<isOnBridgeid<<std::endl;
  if (isOnBridgeFlag)
  {
    double laneWidth = m_manager.getLaneWidth();
    double RoadWidth;
    double RoadWidthMax=5;
    bool endwhileFlag=0;
    posInfo posInfo1=m_posInfo;
    posInfo posInfo2=m_posInfo;
    OpenDrive::Lane *laneInfonext = posInfo1.lane;
    OpenDrive::Lane *laneInfolast = posInfo2.lane;
    RoadWidth=posInfo1.laneWidth;
    std::cout << "lanewidth"<< RoadWidth<<std::endl;
    while(((OpenDrive::Lane *)laneInfonext->getRight()))
    {
      OpenDrive::Lane *nextLaneInfo1 = (OpenDrive::Lane *)laneInfonext->getRight();
      posInfo1.lane = nextLaneInfo1;
      posInfo1.laneId= posInfo1.lane->mId;
      laneInfonext= nextLaneInfo1;
      std::cout << "getRight:"<< std::endl;
      if (laneInfonext->mType==((OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_TAXI)))
      {
        m_manager.setLanePos(posInfo1.roadId,posInfo1.laneId , posInfo1.trackS);
        m_manager.lane2inertial();
        posInfo1.laneWidth=m_manager.getLaneWidth();
        RoadWidth=RoadWidth+posInfo1.laneWidth;
        std::cout << "laneId:"<< posInfo1.laneId<<std::endl;
        std::cout << "LaneWidth:"<<posInfo1.laneWidth<<std::endl;
        std::cout << "RoadWidth:"<< RoadWidth<<std::endl;
      }
      if ((RoadWidth>RoadWidthMax))
      {
        return 0 ;
        break ;
      }
      else if((OpenDrive::Lane *)laneInfonext->getRight()==NULL)
      {
        endwhileFlag=1;
        break;
      }
    }
    if (endwhileFlag==1)
    {
      while((OpenDrive::Lane *)laneInfolast->getLeft())
      {
        OpenDrive::Lane *lastLaneInfo1 = (OpenDrive::Lane *)laneInfolast->getLeft();
        m_posInfo.lane = lastLaneInfo1;
        laneInfolast=lastLaneInfo1;
        if (laneInfolast->mType==(OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_BUS)||(OpenDrive::EnLaneType::ODR_LANE_TYPE_TAXI))
        {
          m_manager.setLanePos(posInfo2.roadId,posInfo2.laneId , posInfo2.trackS);
          m_manager.lane2inertial();
          posInfo2.laneWidth=m_manager.getLaneWidth();
          RoadWidth=RoadWidth+posInfo2.laneWidth;
          std::cout << "getLeft:"<< std::endl;
          std::cout << "laneId:"<< m_posInfo.laneId<<std::endl;
          std::cout << "LaneWidth:"<<posInfo2.laneWidth<<std::endl;
          std::cout << "RoadWidth:"<< RoadWidth<<std::endl;
        }
        if (RoadWidth>RoadWidthMax)
        {
          return 0 ;
          break ;
        }
        if ((OpenDrive::Lane *)laneInfolast->getLeft()==NULL)
        {
          break ;
        }
      }
    }
    if (RoadWidth<=RoadWidthMax) 
    {
      return 1 ;
    }
  }
  return 0 ;
}

// //TODO isSideWalk
// //need to test
// bool getPosInfo::isInSec1_47_1(Point p,float heading1,float *SideWalkDis)
// {
//   bool isInRoad = getInertialPosInfo(p);
//   bool alongRoadFlag= getPosInfo::isSameDirectionWithRoad(p,heading1);
//   int SideWalkDisMax=5;
//   //if sidewalk is at front within SideWalkDis meter
//   if (m_posInfo.laneType==OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK)
//   {
//     (*SideWalkDis)=0;
//     return 1 ;
//   }
//   else if (alongRoadFlag)
//   {
//     if (m_posInfo.roadlength-m_posInfo.trackS-SideWalkDisMax>0)
//     {
//       while (m_posInfo.section->getRight())
//       {           
//         OpenDrive::LaneSection *secInfo1 = m_posInfo.section;
//         OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getRight();
//         m_posInfo.section = nextSecInfo1;
//         m_posInfo.sectionS = m_posInfo.section->mS;
//         if (m_posInfo.sectionS-m_posInfo.trackS-SideWalkDisMax>0)
//         {
//           break;
//         }
//         else if (m_posInfo.lane->getSuccessorLink())
//         {
//           m_posInfo.lane = m_posInfo.lane->getSuccessor();
//           m_posInfo.laneId = m_posInfo.lane->mId;
//           if (m_posInfo.laneType==OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK)
//           {
//             (*SideWalkDis)=m_posInfo.roadlength-m_posInfo.trackS;
//             return 1 ;
//           }
//           std::cout << "20pos_info1.laneId " <<m_posInfo.laneId<< std::endl;
//         }
//         else
//         {
//           break ;
//         }
//       }
//     }
//   }
//   else 
//   {
//     if (m_posInfo.trackS-SideWalkDisMax>0)
//     {
//       while (m_posInfo.section->getLeft())
//       {           
//         OpenDrive::LaneSection *secInfo1 = m_posInfo.section;
//         OpenDrive::LaneSection *nextSecInfo1 = (OpenDrive::LaneSection *)secInfo1->getLeft();
//         m_posInfo.section = nextSecInfo1;
//         m_posInfo.sectionS = m_posInfo.section->mS;
//         if (m_posInfo.sectionS-m_posInfo.trackS-SideWalkDisMax>0)
//         {
//           break;
//         }
//         else if (m_posInfo.lane->getSuccessorLink())
//         {
//           m_posInfo.lane = m_posInfo.lane->getSuccessor();
//           m_posInfo.laneId = m_posInfo.lane->mId;
//           if (m_posInfo.laneType==OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK)
//           {
//             (*SideWalkDis)=m_posInfo.trackS;
//             return 1 ;
//           }
//           std::cout << "20pos_info1.laneId " <<m_posInfo.laneId<< std::endl;
//         }
//         else
//         {
//           break ;
//         }
//       }
//     }
//   }
//   return 0;
// }

//wh
bool getPosInfo::isInSpecialRoadType(Point p, OpenDrive::EnRoadType roadType) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    if (m_manager.getRoadType() == roadType)
        return true;
    return false;
}

//wh
bool getPosInfo::isInSce2_81_x(Point p) {
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    return isInSpecialRoadType(p, ODR_ROAD_TYPE_MOTORWAY);
}

//jm
posInfo getPosInfo::getInfo() const
{
  return m_posInfo;
}
