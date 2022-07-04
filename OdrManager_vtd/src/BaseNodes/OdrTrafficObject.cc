
#include "OdrTrafficObject.hh"
#include "OdrRoadHeader.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{TrafficObject::TrafficObject():Node(
"\x54\x72\x61\x66\x66\x69\x63\x4f\x62\x6a\x65\x63\x74"){mOpcode=
ODR_OPCODE_TRAFFIC_OBJECT;mLevel=3;}TrafficObject::TrafficObject(TrafficObject*
Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mId=Nf2Ao->mId;}TrafficObject::TrafficObject(
const double&BJBDA,const int&id):Node(
"\x54\x72\x61\x66\x66\x69\x63\x4f\x62\x6a\x65\x63\x74"),mS(BJBDA),mId(id){
mOpcode=ODR_OPCODE_TRAFFIC_OBJECT;mLevel=3;}TrafficObject::~TrafficObject(){}
void TrafficObject::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x69\x64\x3a\x20\x25\x64" "\n",
mId);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x25\x2e\x38\x6c\x66" "\n"
,mS);}RoadHeader*TrafficObject::getRoadHeader(){Node*parent=getParent();if(!
parent)return 0;parent=parent->getParent();if(!parent)return 0;return 
reinterpret_cast<RoadHeader*>(parent->getParent());}LaneSection*TrafficObject::
getLaneSection(){Node*parent=getParent();if(!parent)return 0;return 
reinterpret_cast<LaneSection*>(parent->getParent());}Lane*TrafficObject::getLane
(){return reinterpret_cast<Lane*>(getParent());}Node*TrafficObject::getCopy(bool
 Mupxf){Node*dzamm=new TrafficObject(this);if(Mupxf)deepCopy(dzamm);return dzamm
;}}
