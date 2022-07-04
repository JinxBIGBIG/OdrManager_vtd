
#include "OdrRoadType.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadHeader.hh"
#include <stdio.h>
namespace OpenDrive{RoadType::RoadType():Node("\x52\x6f\x61\x64\x54\x79\x70\x65"
){mOpcode=ODR_OPCODE_ROAD_TYPE;mLevel=1;}RoadType::RoadType(RoadType*Nf2Ao):Node
(Nf2Ao){mStartPos=Nf2Ao->mStartPos;mType=Nf2Ao->mType;mCountry=Nf2Ao->mCountry;
mEndPos=Nf2Ao->mEndPos;}RoadType::~RoadType(){}void RoadType::printData()const{
fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x34\x66" "\n",
mStartPos);fprintf(stderr,
"\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x25\x64" "\n",mType);
fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x75\x6e\x74\x72\x79\x3a\x20\x3c\x25\x73\x3e" "\n",
mCountry.c_str());}bool RoadType::read(ReaderXML*F3vnM){mStartPos=F3vnM->
getDouble("\x73");mType=F3vnM->getOpcodeFromRoadType(F3vnM->getString(
"\x74\x79\x70\x65"));mCountry=F3vnM->getString("\x63\x6f\x75\x6e\x74\x72\x79");
RoadHeader*hdr=reinterpret_cast<RoadHeader*>(getParent());if(hdr)mEndPos=hdr->
mLength;RoadType*a0xD4=reinterpret_cast<RoadType*>(getLeft());if(a0xD4)a0xD4->
mEndPos=mStartPos;return true;}Node*RoadType::getCopy(bool Mupxf){Node*dzamm=new
 RoadType(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
