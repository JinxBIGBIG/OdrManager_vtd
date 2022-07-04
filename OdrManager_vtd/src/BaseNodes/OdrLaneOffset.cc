
#include "OdrLaneOffset.hh"
#include "OdrRoadHeader.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{LaneOffset::LaneOffset():Node(
"\x4c\x61\x6e\x65\x4f\x66\x66\x73\x65\x74"),mSEnd(1.0e20){mOpcode=
ODR_OPCODE_LANE_OFFSET;mLevel=2;}LaneOffset::LaneOffset(LaneOffset*Nf2Ao):Node(
Nf2Ao){mS=Nf2Ao->mS;mA=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;mSEnd=
Nf2Ao->mSEnd;}LaneOffset::~LaneOffset(){}void LaneOffset::printData()const{
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66\x20\x2d\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mS,mSEnd);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x41\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mA);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x42\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mB);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x43\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mC);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x44\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mD);}bool LaneOffset::read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mA=
F3vnM->getDouble("\x61");mB=F3vnM->getDouble("\x62");mC=F3vnM->getDouble("\x63")
;mD=F3vnM->getDouble("\x64");LaneOffset*sJh8U=reinterpret_cast<LaneOffset*>(
getLeft());if(sJh8U)sJh8U->mSEnd=mS;return true;}double LaneOffset::s2offset(
const double&BJBDA){double rganP=BJBDA-mS;if(rganP<0.0)return 0.0;return mA+mB*
rganP+mC*rganP*rganP+mD*rganP*rganP*rganP;}double LaneOffset::s2offsetDot(const 
double&BJBDA){double rganP=BJBDA-mS;if(rganP<0.0)return 0.0;return mB+2.0*mC*
rganP+3.0*mD*rganP*rganP;}double LaneOffset::getMaxValue(){double Cw4z_=s2offset
(mS);double weaNH=s2offset(mSEnd);return(fabs(Cw4z_)>fabs(weaNH))?fabs(Cw4z_):
fabs(weaNH);}void LaneOffset::calcPrepareData(){LaneOffset*Gv01F=
reinterpret_cast<LaneOffset*>(getRight());RoadHeader*hdr=reinterpret_cast<
RoadHeader*>(getParent());mSEnd=Gv01F?Gv01F->mS:hdr->mLength;}void LaneOffset::
calcPostPrepareData(){}Node*LaneOffset::getCopy(bool Mupxf){Node*dzamm=new 
LaneOffset(this);if(Mupxf)deepCopy(dzamm);return dzamm;}double LaneOffset::
s2curvature(const double&BJBDA){double rganP=BJBDA-mS;if(rganP<0.0)return 0.0;
double lgWm4=mB+2.0*mC*rganP+3.0*mD*rganP*rganP;double sj8U1=2.0*mC+6.0*mD*rganP
;double value=1+lgWm4*lgWm4;value=value*value*value;return sj8U1/value;}}
