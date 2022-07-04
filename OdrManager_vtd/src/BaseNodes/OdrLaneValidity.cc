
#include "OdrLaneValidity.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneValidity::LaneValidity():Node(
"\x4c\x61\x6e\x65\x56\x61\x6c\x69\x64\x69\x74\x79"){mOpcode=
ODR_OPCODE_LANE_VALIDITY;mLevel=2;}LaneValidity::LaneValidity(LaneValidity*Nf2Ao
):Node(Nf2Ao){mMinLane=Nf2Ao->mMinLane;mMaxLane=Nf2Ao->mMaxLane;}LaneValidity::~
LaneValidity(){}void LaneValidity::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x69\x6e\x2e\x20\x6c\x61\x6e\x65\x3a\x20\x25\x64" "\n"
,mMinLane);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x61\x78\x2e\x20\x6c\x61\x6e\x65\x3a\x20\x25\x64" "\n"
,mMaxLane);}bool LaneValidity::read(ReaderXML*F3vnM){mMinLane=F3vnM->getInt(
"\x66\x72\x6f\x6d\x4c\x61\x6e\x65");mMaxLane=F3vnM->getInt(
"\x74\x6f\x4c\x61\x6e\x65");return true;}Node*LaneValidity::getCopy(bool Mupxf){
Node*dzamm=new LaneValidity(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
