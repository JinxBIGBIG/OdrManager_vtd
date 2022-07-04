
#include "OdrRoadMarkType.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{RoadMarkType::RoadMarkType():Node(
"\x52\x6f\x61\x64\x4d\x61\x72\x6b\x54\x79\x70\x65"){mOpcode=
ODR_OPCODE_ROAD_MARK_TYPE;mLevel=4;mName=std::string(
"\x73\x74\x61\x6e\x64\x61\x72\x64");mWidth=0.0;mHasWidth=false;}RoadMarkType::
RoadMarkType(RoadMarkType*Nf2Ao):Node(Nf2Ao){mName=Nf2Ao->mName;mWidth=Nf2Ao->
mWidth;mHasWidth=Nf2Ao->mHasWidth;}RoadMarkType::~RoadMarkType(){}void 
RoadMarkType::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6e\x61\x6d\x65\x3a\x20\x20\x20\x20\x25\x73" "\n"
,mName.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x25\x2e\x33\x6c\x66" "\n"
,mWidth);}bool RoadMarkType::read(ReaderXML*F3vnM){mName=F3vnM->getString(
"\x6e\x61\x6d\x65");mWidth=F3vnM->getDouble("\x77\x69\x64\x74\x68");const 
ReaderXML::AttribMap&BMyX5=F3vnM->getAllAttributes();mHasWidth=(BMyX5.find(
"\x77\x69\x64\x74\x68")!=BMyX5.end());return true;}Node*RoadMarkType::getCopy(
bool Mupxf){Node*dzamm=new RoadMarkType(this);if(Mupxf)deepCopy(dzamm);return 
dzamm;}}
