
#include "OdrRoadHeader.hh"
#include "OdrJuncLink.hh"
#include "OdrJuncLaneLink.hh"
#include "OdrGeoNode.hh"
#include "OdrGeoHeader.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{JuncLink::JuncLink():Node("\x4a\x75\x6e\x63\x4c\x69\x6e\x6b"
),mIncomingRoad(0),mConnectingRoad(0),mTurnAngle(0.0){mOpcode=
ODR_OPCODE_JUNCTION_LINK;mLevel=1;}JuncLink::JuncLink(JuncLink*Nf2Ao):Node(Nf2Ao
){mId=Nf2Ao->mId;mIdAsString=Nf2Ao->mIdAsString;mRoadId=Nf2Ao->mRoadId;
mRoadIdAsString=Nf2Ao->mRoadIdAsString;mPathId=Nf2Ao->mPathId;mPathIdAsString=
Nf2Ao->mPathIdAsString;mDir=Nf2Ao->mDir;mIncomingRoad=Nf2Ao->mIncomingRoad;
mConnectingRoad=Nf2Ao->mConnectingRoad;mIsVirtual=Nf2Ao->mIsVirtual;
mConnectionMasterId=Nf2Ao->mConnectionMasterId;mConnectionMaster=Nf2Ao->
mConnectionMaster;}JuncLink::~JuncLink(){}void JuncLink::printData()const{
fprintf(stderr,
"\x20\x20\x20\x20\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x52\x6f\x61\x64\x20\x49\x44\x3a\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mRoadIdAsString.c_str(),mRoadId);fprintf(stderr,
"\x20\x20\x20\x20\x50\x61\x74\x68\x20\x49\x44\x3a\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mPathIdAsString.c_str(),mPathId);fprintf(stderr,
"\x20\x20\x20\x20\x44\x69\x72\x65\x63\x74\x69\x6f\x6e\x3a\x20\x25\x64" "\n",mDir
);fprintf(stderr,
"\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e" "\n"
,mIsVirtual?"\x76\x69\x72\x74\x75\x61\x6c":"\x64\x65\x66\x61\x75\x6c\x74");
fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x6e\x6e\x65\x63\x74\x69\x6f\x6e\x4d\x61\x73\x74\x65\x72\x3a\x20\x3c\x25\x73\x3e" "\n"
,mConnectionMasterId.c_str());}bool JuncLink::read(ReaderXML*F3vnM){mId=F3vnM->
getUInt("\x69\x64");mIdAsString=F3vnM->getString("\x69\x64");mRoadId=F3vnM->
getUInt("\x69\x6e\x63\x6f\x6d\x69\x6e\x67\x52\x6f\x61\x64");mRoadIdAsString=
F3vnM->getString("\x69\x6e\x63\x6f\x6d\x69\x6e\x67\x52\x6f\x61\x64");mPathId=
F3vnM->getUInt("\x63\x6f\x6e\x6e\x65\x63\x74\x69\x6e\x67\x52\x6f\x61\x64");
mPathIdAsString=F3vnM->getString(
"\x63\x6f\x6e\x6e\x65\x63\x74\x69\x6e\x67\x52\x6f\x61\x64");mDir=F3vnM->
getString("\x63\x6f\x6e\x74\x61\x63\x74\x50\x6f\x69\x6e\x74")==
"\x73\x74\x61\x72\x74"?ODR_LINK_POINT_START:ODR_LINK_POINT_END;mIsVirtual=F3vnM
->getString("\x74\x79\x70\x65")==std::string("\x76\x69\x72\x74\x75\x61\x6c");
mConnectionMasterId=F3vnM->getString(
"\x63\x6f\x6e\x6e\x65\x63\x74\x69\x6f\x6e\x4d\x61\x73\x74\x65\x72");
mConnectionMaster=0;return true;}void JuncLink::calcPrepareData(){if(!RoadData::
getInstance()){fprintf(stderr,
"\x4a\x75\x6e\x63\x4c\x69\x6e\x6b\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21" "\n" "\x20"
);return;}mIncomingRoad=reinterpret_cast<RoadHeader*>(RoadData::getInstance()->
getNodeFromId(ODR_OPCODE_ROAD_HEADER,mRoadIdAsString));mConnectingRoad=
reinterpret_cast<RoadHeader*>(RoadData::getInstance()->getNodeFromId(
ODR_OPCODE_ROAD_HEADER,mPathIdAsString));if(mConnectingRoad){mConnectingRoad->
setJunction(getParent());GeoHeader*xjOaT=mConnectingRoad->getFirstGeoHeader();
double j5OQN=0.0;double kksjr=0.0;if(xjOaT)j5OQN=xjOaT->mH;xjOaT=mConnectingRoad
->getLastGeoHeader();if(xjOaT)kksjr=xjOaT->mHEnd;mTurnAngle=(mDir==
ODR_LINK_POINT_START?1.0:-1.0)*(kksjr-j5OQN);while(mTurnAngle<-M_PI)mTurnAngle+=
2.0*M_PI;while(mTurnAngle>M_PI)mTurnAngle-=2.0*M_PI;}if(mIncomingRoad)
mIncomingRoad->calcPostPrepareData();if(mConnectionMasterId!=std::string(
"\x75\x6e\x6b\x6e\x6f\x77\x6e")){JuncLink*hYfdO=reinterpret_cast<JuncLink*>(
getParent()->getChild(ODR_OPCODE_JUNCTION_LINK));while(hYfdO&&!mConnectionMaster
){if(mConnectionMasterId==hYfdO->mIdAsString)mConnectionMaster=hYfdO;hYfdO=
reinterpret_cast<JuncLink*>(hYfdO->getRight());}}}JuncLaneLink*JuncLink::
getFirstLaneLink(){return reinterpret_cast<JuncLaneLink*>(getChild(
ODR_OPCODE_JUNCTION_LANE_LINK));}}
