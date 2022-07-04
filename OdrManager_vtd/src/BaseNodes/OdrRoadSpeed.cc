
#include "OdrRoadSpeed.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadHeader.hh"
#include <stdio.h>
const double OpenDrive::RoadSpeed::scMph2ms=1.0/2.2369362920544;const double 
OpenDrive::RoadSpeed::scKmh2ms=1.0/3.6;namespace OpenDrive{RoadSpeed::RoadSpeed(
):Node("\x52\x6f\x61\x64\x53\x70\x65\x65\x64"){mOpcode=ODR_OPCODE_ROAD_SPEED;
mLevel=2;}RoadSpeed::RoadSpeed(RoadSpeed*Nf2Ao):Node(Nf2Ao){mMax=Nf2Ao->mMax;
mUnit=Nf2Ao->mUnit;}RoadSpeed::~RoadSpeed(){}void RoadSpeed::printData()const{
fprintf(stderr,"\x20\x20\x20\x20\x6d\x61\x78\x3a\x20\x20\x25\x2e\x34\x66" "\n",
mMax);fprintf(stderr,"\x20\x20\x20\x20\x75\x6e\x69\x74\x3a\x20\x25\x64" "\n",
mUnit);}bool RoadSpeed::read(ReaderXML*F3vnM){mMax=F3vnM->getDouble(
"\x6d\x61\x78");mUnit=F3vnM->getOpcodeFromUnit(F3vnM->getString(
"\x75\x6e\x69\x74"));if(mMax==-1.0)mMax=1.0e10;else if((F3vnM->getString(
"\x6d\x61\x78")=="\x6e\x6f\x20\x6c\x69\x6d\x69\x74")||(F3vnM->getString(
"\x6d\x61\x78")=="\x75\x6e\x64\x65\x66\x69\x6e\x65\x64"))mMax=1.0e10;else{if(
mUnit==ODR_UNIT_SPEED_MPH)mMax*=scMph2ms;else if(mUnit==ODR_UNIT_SPEED_KMH)mMax
*=scKmh2ms;}mUnit=ODR_UNIT_SPEED_MPS;return true;}Node*RoadSpeed::getCopy(bool 
Mupxf){Node*dzamm=new RoadSpeed(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
