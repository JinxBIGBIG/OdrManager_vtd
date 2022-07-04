
#include "OdrObjectRef.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OdrRoadHeader.hh"
#include "OdrObject.hh"
#include "OdrLaneValidity.hh"
#include <stdio.h>
namespace OpenDrive{ObjectRef::ObjectRef():Node(
"\x4f\x62\x6a\x65\x63\x74\x52\x65\x66"){mOpcode=ODR_OPCODE_OBJECT_REFERENCE;
mLevel=1;mObject=0;}ObjectRef::ObjectRef(ObjectRef*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->
mS;mT=Nf2Ao->mT;mId=Nf2Ao->mId;mIdAsString=Nf2Ao->mIdAsString;mDir=Nf2Ao->mDir;
mObject=Nf2Ao->mObject;mZ=Nf2Ao->mZ;mLength=Nf2Ao->mLength;mMinLane=Nf2Ao->
mMinLane;mMaxLane=Nf2Ao->mMaxLane;}ObjectRef::ObjectRef(Object*QAAX5):Node(
"\x4f\x62\x6a\x65\x63\x74\x52\x65\x66"){mOpcode=ODR_OPCODE_OBJECT_REFERENCE;
mLevel=1;mObject=0;mS=QAAX5->mS;mT=QAAX5->mT;mId=QAAX5->mId;mIdAsString=QAAX5->
mIdAsString;mDir=QAAX5->mDir;mZ=QAAX5->mZ;mLength=QAAX5->mLength;mObject=QAAX5;}
ObjectRef::~ObjectRef(){}void ObjectRef::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mT);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x64\x69\x72\x65\x63\x74\x69\x6f\x6e\x3a\x20\x20\x25\x64" "\n"
,mDir);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x4f\x62\x6a\x65\x63\x74\x3a\x20\x20\x20\x20\x25\x70" "\n"
,mObject);}bool ObjectRef::read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mT=
F3vnM->getDouble("\x74");mId=F3vnM->getUInt("\x69\x64");mIdAsString=F3vnM->
getString("\x69\x64");mDir=F3vnM->getString(
"\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2b"?ODR_DIRECTION_PLUS:(
F3vnM->getString("\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2d"?
ODR_DIRECTION_MINUS:ODR_DIRECTION_NONE);mZ=F3vnM->getDouble(
"\x7a\x4f\x66\x66\x73\x65\x74");mLength=F3vnM->getDouble(
"\x76\x61\x6c\x69\x64\x4c\x65\x6e\x67\x74\x68");return true;}Node*ObjectRef::
getCopy(bool Mupxf){Node*dzamm=new ObjectRef(this);if(Mupxf)deepCopy(dzamm);
return dzamm;}const double&ObjectRef::getS()const{return mS;}void ObjectRef::
calcPrepareData(){if(!RoadData::getInstance()){fprintf(stderr,
"\x4f\x62\x6a\x65\x63\x74\x52\x65\x66\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21" "\n" "\x20"
);return;}mObject=reinterpret_cast<Object*>(RoadData::getInstance()->
getNodeFromId(ODR_OPCODE_OBJECT,mIdAsString));LaneValidity*aVXDm=
reinterpret_cast<LaneValidity*>(getChild(ODR_OPCODE_LANE_VALIDITY));if(aVXDm){
mMinLane=aVXDm->mMinLane;mMaxLane=aVXDm->mMaxLane;}}}
