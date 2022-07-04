
#include "OdrSignalRef.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OdrRoadHeader.hh"
#include "OdrSignal.hh"
#include "OdrLaneValidity.hh"
#include <stdio.h>
namespace OpenDrive{SignalRef::SignalRef():Node(
"\x53\x69\x67\x6e\x61\x6c\x52\x65\x66"){mOpcode=ODR_OPCODE_SIGNAL_REFERENCE;
mLevel=1;mSignal=0;}SignalRef::SignalRef(SignalRef*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->
mS;mT=Nf2Ao->mT;mId=Nf2Ao->mId;mIdAsString=Nf2Ao->mIdAsString;mDir=Nf2Ao->mDir;
mSignal=Nf2Ao->mSignal;mMinLane=Nf2Ao->mMinLane;mMaxLane=Nf2Ao->mMaxLane;}
SignalRef::SignalRef(Signal*kyJQa):Node("\x53\x69\x67\x6e\x61\x6c\x52\x65\x66"){
mOpcode=ODR_OPCODE_SIGNAL_REFERENCE;mLevel=1;mSignal=0;mS=kyJQa->mS;mT=kyJQa->mT
;mId=kyJQa->mId;mIdAsString=kyJQa->mIdAsString;mDir=kyJQa->mDir;mLineNo=kyJQa->
getLineNo();mDebugInfo=kyJQa->getDebugInfo();mMinLane=kyJQa->mMinLane;mMaxLane=
kyJQa->mMaxLane;}SignalRef::~SignalRef(){}void SignalRef::printData()const{
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mT);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x64\x69\x72\x65\x63\x74\x69\x6f\x6e\x3a\x20\x20\x25\x64" "\n"
,mDir);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x53\x69\x67\x6e\x61\x6c\x3a\x20\x20\x20\x20\x30\x78\x25\x6c\x78" "\n"
,(unsigned long long)mSignal);}bool SignalRef::read(ReaderXML*F3vnM){mS=F3vnM->
getDouble("\x73");mT=F3vnM->getDouble("\x74");mId=F3vnM->getUInt("\x69\x64");
mIdAsString=F3vnM->getString("\x69\x64");mDir=F3vnM->getString(
"\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2b"?ODR_DIRECTION_PLUS:(
F3vnM->getString("\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2d"?
ODR_DIRECTION_MINUS:ODR_DIRECTION_NONE);mMinLane=-99;mMaxLane=99;return true;}
Node*SignalRef::getCopy(bool Mupxf){Node*dzamm=new SignalRef(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}const double&SignalRef::getS()const{return mS;}
void SignalRef::calcPrepareData(){if(!RoadData::getInstance()){fprintf(stderr,
"\x53\x69\x67\x6e\x61\x6c\x52\x65\x66\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21" "\n" "\x20"
);return;}mSignal=reinterpret_cast<Signal*>(RoadData::getInstance()->
getNodeFromId(ODR_OPCODE_SIGNAL,mIdAsString));LaneValidity*aVXDm=
reinterpret_cast<LaneValidity*>(getChild(ODR_OPCODE_LANE_VALIDITY));if(aVXDm){
mMinLane=aVXDm->mMinLane;mMaxLane=aVXDm->mMaxLane;}}}
