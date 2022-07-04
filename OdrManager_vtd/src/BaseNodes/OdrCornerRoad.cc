
#include "OdrCornerRoad.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{CornerRoad::CornerRoad():Node(
"\x43\x6f\x72\x6e\x65\x72\x52\x6f\x61\x64"){mOpcode=ODR_OPCODE_CORNER_ROAD;
mLevel=4;}CornerRoad::CornerRoad(CornerRoad*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mT=
Nf2Ao->mT;mDz=Nf2Ao->mDz;mHeight=Nf2Ao->mHeight;mId=Nf2Ao->mId;}CornerRoad::~
CornerRoad(){}void CornerRoad::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x64" "\n",
mId);fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mT);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mDz);fprintf(stderr,
"\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHeight);}bool CornerRoad::read(ReaderXML*F3vnM){mId=F3vnM->getInt("\x69\x64");
mS=F3vnM->getDouble("\x73");mT=F3vnM->getDouble("\x74");mDz=F3vnM->getDouble(
"\x64\x7a");mHeight=F3vnM->getDouble("\x68\x65\x69\x67\x68\x74");return true;}
Node*CornerRoad::getCopy(bool Mupxf){Node*dzamm=new CornerRoad(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}}
