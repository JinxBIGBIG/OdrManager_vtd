
#include "OdrJuncLaneLink.hh"
#include "OdrLane.hh"
#include "OdrLaneSection.hh"
#include "OdrJuncHeader.hh"
#include "OdrJuncLink.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{JuncLaneLink::JuncLaneLink():Node(
"\x4a\x75\x6e\x63\x4c\x61\x6e\x65\x4c\x69\x6e\x6b"),mIncomingLane(0),
mConnectingLane(0){mOpcode=ODR_OPCODE_JUNCTION_LANE_LINK;mLevel=2;}JuncLaneLink
::~JuncLaneLink(){}void JuncLaneLink::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x61\x64\x20\x4c\x61\x6e\x65\x3a\x20\x25\x64" "\n"
,mRoadLane);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x50\x61\x74\x68\x20\x4c\x61\x6e\x65\x3a\x20\x25\x64" "\n"
,mPathLane);}bool JuncLaneLink::read(ReaderXML*F3vnM){mRoadLane=F3vnM->getInt(
"\x66\x72\x6f\x6d");mPathLane=F3vnM->getInt("\x74\x6f");return true;}void 
JuncLaneLink::calcPrepareData(){RoadHeader*inRoad=(reinterpret_cast<JuncLink*>(
getParent()))->mIncomingRoad;RoadHeader*connRoad=(reinterpret_cast<JuncLink*>(
getParent()))->mConnectingRoad;if(!inRoad||!connRoad)return;mIncomingLane=0;
mConnectingLane=0;LaneSection*H4prs=0;JuncHeader*GqNs7=reinterpret_cast<
JuncHeader*>(getParent()->getParent());if(inRoad->getPredecessorNode()==GqNs7)
H4prs=inRoad->getFirstLaneSection();else H4prs=inRoad->getLastLaneSection();if(!
H4prs){fprintf(stderr,
"\x46\x41\x54\x41\x4c\x3a\x20\x52\x6f\x61\x64\x20\x25\x75\x20\x68\x61\x73\x20\x6e\x6f\x20\x6c\x61\x6e\x65\x20\x73\x65\x63\x74\x69\x6f\x6e\x2e" "\n"
,inRoad->mId);return;}Lane*lane=reinterpret_cast<Lane*>(H4prs->getChild());while
(lane&&!mIncomingLane){if(lane->mId==mRoadLane)mIncomingLane=lane;lane=
reinterpret_cast<Lane*>(lane->getRight());}if((reinterpret_cast<JuncLink*>(
getParent()))->mDir==ODR_LINK_POINT_START)H4prs=connRoad->getFirstLaneSection();
else H4prs=connRoad->getLastLaneSection();if(!H4prs){fprintf(stderr,
"\x46\x41\x54\x41\x4c\x3a\x20\x43\x6f\x6e\x6e\x65\x63\x74\x69\x6f\x6e\x20\x72\x6f\x61\x64\x20\x25\x75\x20\x68\x61\x73\x20\x6e\x6f\x20\x6c\x61\x6e\x65\x20\x73\x65\x63\x74\x69\x6f\x6e\x2e" "\n"
,connRoad->mId);return;}lane=reinterpret_cast<Lane*>(H4prs->getChild());while(
lane&&!mConnectingLane){if(lane->mId==mPathLane)mConnectingLane=lane;lane=
reinterpret_cast<Lane*>(lane->getRight());}}}
