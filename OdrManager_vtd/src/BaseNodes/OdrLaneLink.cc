
#include "OdrLaneLink.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneLink::LaneLink():Node("\x4c\x61\x6e\x65\x4c\x69\x6e\x6b"
){mOpcode=ODR_OPCODE_LANE_LINK;mLevel=3;}LaneLink::LaneLink(LaneLink*Nf2Ao):Node
(Nf2Ao){mType=Nf2Ao->mType;mLaneId=Nf2Ao->mLaneId;}LaneLink::LaneLink(unsigned 
short type):Node("\x4c\x61\x6e\x65\x4c\x69\x6e\x6b"){mOpcode=
ODR_OPCODE_LANE_LINK;mLevel=3;mType=type;}LaneLink::~LaneLink(){}void LaneLink::
printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x25\x64" "\n"
,mType);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4c\x61\x6e\x65\x20\x49\x44\x3a\x20\x25\x64" "\n"
,mLaneId);}bool LaneLink::read(ReaderXML*F3vnM){mLaneId=F3vnM->getInt("\x69\x64"
);return true;}Node*LaneLink::getCopy(bool Mupxf){Node*dzamm=new LaneLink(this);
if(Mupxf)deepCopy(dzamm);return dzamm;}}
