
#include "OdrParkingSpaceMarking.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{ParkingSpaceMarking::ParkingSpaceMarking():Node(
"\x50\x61\x72\x6b\x69\x6e\x67\x53\x70\x61\x63\x65\x4d\x61\x72\x6b\x69\x6e\x67"){
mOpcode=ODR_OPCODE_PARKING_SPACE_MARKING;mLevel=3;}ParkingSpaceMarking::
ParkingSpaceMarking(ParkingSpaceMarking*Nf2Ao):Node(Nf2Ao){mSide=Nf2Ao->mSide;
mType=Nf2Ao->mType;mWidth=Nf2Ao->mWidth;mColor=Nf2Ao->mColor;}
ParkingSpaceMarking::~ParkingSpaceMarking(){}void ParkingSpaceMarking::printData
()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x69\x64\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mSide.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x74\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mType);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mWidth);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x43\x6f\x6c\x6f\x72\x3a\x20\x20\x20\x20\x25\x64" "\n"
,mColor);}bool ParkingSpaceMarking::read(ReaderXML*F3vnM){mSide=F3vnM->getString
("\x73\x69\x64\x65");mType=F3vnM->getOpcodeFromRoadMarkType(F3vnM->getString(
"\x74\x79\x70\x65"));mWidth=F3vnM->getDouble("\x77\x69\x64\x74\x68");mColor=
F3vnM->getOpcodeFromRoadMarkColor(F3vnM->getString("\x63\x6f\x6c\x6f\x72"));
return true;}Node*ParkingSpaceMarking::getCopy(bool Mupxf){Node*dzamm=new 
ParkingSpaceMarking(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
