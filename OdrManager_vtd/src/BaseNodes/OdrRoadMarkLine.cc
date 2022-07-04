
#include "OdrRoadMarkLine.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{RoadMarkLine::RoadMarkLine():Node(
"\x52\x6f\x61\x64\x4d\x61\x72\x6b\x4c\x69\x6e\x65"){mOpcode=
ODR_OPCODE_ROAD_MARK_LINE;mLevel=5;mLength=0.0;mSpace=0.0;mOffsetT=0.0;mOffsetS=
0.0;mRule=0.0;mWidth=0.0;mColor=std::string("\x77\x68\x69\x74\x65");mHasWidth=
true;mHasColor=false;}RoadMarkLine::RoadMarkLine(RoadMarkLine*Nf2Ao):Node(Nf2Ao)
{mLength=Nf2Ao->mLength;mSpace=Nf2Ao->mSpace;mOffsetT=Nf2Ao->mOffsetT;mOffsetS=
Nf2Ao->mOffsetS;mRule=Nf2Ao->mRule;mWidth=Nf2Ao->mWidth;mHasWidth=Nf2Ao->
mHasWidth;mColor=Nf2Ao->mColor;mHasColor=Nf2Ao->mHasColor;}RoadMarkLine::~
RoadMarkLine(){}void RoadMarkLine::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6c\x65\x6e\x67\x74\x68\x3a\x20\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mLength);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x70\x61\x63\x65\x3a\x20\x20\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mSpace);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x74\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mOffsetT);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mOffsetS);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x72\x75\x6c\x65\x3a\x20\x20\x20\x20\x20\x25\x64" "\n"
,mRule);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mWidth);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x63\x6f\x6c\x6f\x72\x3a\x20\x20\x20\x20\x3c\x25\x73\x3e" "\n"
,mColor.c_str());}bool RoadMarkLine::read(ReaderXML*F3vnM){mLength=F3vnM->
getDouble("\x6c\x65\x6e\x67\x74\x68");mSpace=F3vnM->getDouble(
"\x73\x70\x61\x63\x65");mOffsetT=F3vnM->getDouble("\x74\x4f\x66\x66\x73\x65\x74"
);mOffsetS=F3vnM->getDouble("\x73\x4f\x66\x66\x73\x65\x74");mRule=F3vnM->
getOpcodeFromRoadMarkRule(F3vnM->getString("\x72\x75\x6c\x65"));mWidth=F3vnM->
getDouble("\x77\x69\x64\x74\x68");mColor=F3vnM->getString("\x63\x6f\x6c\x6f\x72"
);const ReaderXML::AttribMap&BMyX5=F3vnM->getAllAttributes();mHasWidth=(BMyX5.
find("\x77\x69\x64\x74\x68")!=BMyX5.end());mHasColor=(BMyX5.find(
"\x63\x6f\x6c\x6f\x72")!=BMyX5.end());return true;}Node*RoadMarkLine::getCopy(
bool Mupxf){Node*dzamm=new RoadMarkLine(this);if(Mupxf)deepCopy(dzamm);return 
dzamm;}}
