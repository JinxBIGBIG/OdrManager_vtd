
#include "OdrLaneSpeed.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneSpeed::LaneSpeed():Node(
"\x4c\x61\x6e\x65\x53\x70\x65\x65\x64"),mUnit(ODR_UNIT_SPEED_MPS){mOpcode=
ODR_OPCODE_LANE_SPEED;mLevel=3;}LaneSpeed::LaneSpeed(LaneSpeed*Nf2Ao):Node(Nf2Ao
){mOffset=Nf2Ao->mOffset;mSpeed=Nf2Ao->mSpeed;mUnit=Nf2Ao->mUnit;}LaneSpeed::~
LaneSpeed(){}void LaneSpeed::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOffset);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x61\x78\x2e\x20\x53\x70\x65\x65\x64\x3a\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mSpeed);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x75\x6e\x69\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mUnit);}bool LaneSpeed::read(ReaderXML*F3vnM){mOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");mSpeed=F3vnM->getDouble("\x6d\x61\x78");mUnit=
F3vnM->getOpcodeFromUnit(F3vnM->getString("\x75\x6e\x69\x74"));return true;}
double LaneSpeed::ds2speed(const double&rganP){double BJBDA=rganP-mOffset;if(
BJBDA<0.0)return 0.0;if(mUnit==ODR_UNIT_SPEED_KMH)return mSpeed/3.6;if(mUnit==
ODR_UNIT_SPEED_MPH)return mSpeed*1609.344/3600.0;return mSpeed;}Node*LaneSpeed::
getCopy(bool Mupxf){Node*dzamm=new LaneSpeed(this);if(Mupxf)deepCopy(dzamm);
return dzamm;}}
