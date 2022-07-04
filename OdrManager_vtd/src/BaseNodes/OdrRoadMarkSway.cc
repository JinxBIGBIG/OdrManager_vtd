
#include "OdrRoadMarkSway.hh"
#include "OdrRoadMark.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{RoadMarkSway::RoadMarkSway():Node(
"\x52\x6f\x61\x64\x4d\x61\x72\x6b\x53\x77\x61\x79"){mOpcode=
ODR_OPCODE_ROAD_MARK_SWAY;mLevel=4;}RoadMarkSway::RoadMarkSway(RoadMarkSway*
Nf2Ao):Node(Nf2Ao){mDs=Nf2Ao->mDs;mA=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=
Nf2Ao->mD;mS=Nf2Ao->mS;mSEnd=Nf2Ao->mSEnd;}RoadMarkSway::~RoadMarkSway(){}void 
RoadMarkSway::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x64\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mDs);fprintf(stderr,
"\x20\x20\x20\x20\x61\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mA);fprintf(stderr,
"\x20\x20\x20\x20\x62\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mB);fprintf(stderr,
"\x20\x20\x20\x20\x63\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mC);fprintf(stderr,
"\x20\x20\x20\x20\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mD);}bool RoadMarkSway::read(ReaderXML*F3vnM){mDs=F3vnM->getDouble("\x64\x73");
mA=F3vnM->getDouble("\x61");mB=F3vnM->getDouble("\x62");mC=F3vnM->getDouble(
"\x63");mD=F3vnM->getDouble("\x64");return true;}Node*RoadMarkSway::getCopy(bool
 Mupxf){Node*dzamm=new RoadMarkSway(this);if(Mupxf)deepCopy(dzamm);return dzamm;
}double RoadMarkSway::s2offset(const double&BJBDA){double rganP=BJBDA-mS;return 
mA+mB*rganP+mC*rganP*rganP+mD*rganP*rganP*rganP;}void RoadMarkSway::
calcPrepareData(){RoadMark*ndRaj=reinterpret_cast<RoadMark*>(getParent());if(!
ndRaj)return;mS=ndRaj->mS+mDs;mSEnd=ndRaj->mSEnd;RoadMarkSway*UB_nU=
reinterpret_cast<RoadMarkSway*>(getLeft());if(UB_nU)UB_nU->mSEnd=mS;}}
