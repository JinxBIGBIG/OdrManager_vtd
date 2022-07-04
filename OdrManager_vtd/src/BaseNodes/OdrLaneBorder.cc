
#include "OdrLaneBorder.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{LaneBorder::LaneBorder():Node(
"\x4c\x61\x6e\x65\x42\x6f\x72\x64\x65\x72"),mOffsetEnd(1.0e20),mMinWidth(0.0),
mMaxWidth(0.0),mInnerLane(0),mInnerLaneBorder(0){mOpcode=ODR_OPCODE_LANE_BORDER;
mLevel=3;}LaneBorder::LaneBorder(LaneBorder*Nf2Ao):Node(Nf2Ao){mOffset=Nf2Ao->
mOffset;mA=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;mOffsetEnd=Nf2Ao->
mOffsetEnd;Nf2Ao->getMinMax(mMinWidth,mMaxWidth);}LaneBorder::~LaneBorder(){}
void LaneBorder::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66\x20\x2d\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOffset,mOffsetEnd);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x41\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mA);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x42\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mB);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x43\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mC);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mD);}bool LaneBorder::read(ReaderXML*F3vnM){mOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");mA=F3vnM->getDouble("\x61");mB=F3vnM->getDouble(
"\x62");mC=F3vnM->getDouble("\x63");mD=F3vnM->getDouble("\x64");LaneBorder*Mp7pR
=reinterpret_cast<LaneBorder*>(getLeft());if(Mp7pR)Mp7pR->mOffsetEnd=mOffset;
return true;}double LaneBorder::ds2borderT(const double&rganP){double BJBDA=
rganP-mOffset;return mA+mB*BJBDA+mC*BJBDA*BJBDA+mD*BJBDA*BJBDA*BJBDA;}double 
LaneBorder::ds2borderTDot(const double&rganP){double BJBDA=rganP-mOffset;return 
mB+2.0*mC*BJBDA+3.0*mD*BJBDA*BJBDA;}double LaneBorder::ds2width(const double&
rganP){double lGwJr=ds2borderT(rganP);if(!mInnerLaneBorder){if(!mInnerLane)
return 0.0;return fabs(lGwJr);}LaneBorder*earcr=mInnerLaneBorder;while(earcr&&(
earcr->mOffsetEnd<rganP))earcr=reinterpret_cast<LaneBorder*>(earcr->getRight());
if(!earcr)return fabs(lGwJr);double eTAL0=earcr->ds2borderT(rganP);return fabs(
lGwJr-eTAL0);}double LaneBorder::ds2widthDot(const double&rganP){return 
ds2borderTDot(rganP);}double LaneBorder::ds2Curv(const double&rganP){double 
BJBDA=rganP-mOffset;double gm6Wu=mB+2.0*mC*BJBDA+3.0*mD*BJBDA*BJBDA;gm6Wu*=gm6Wu
;gm6Wu+=1.0;gm6Wu=sqrt(gm6Wu);gm6Wu*=gm6Wu*gm6Wu;return(2.0*mC+6.0*mD*BJBDA)/
gm6Wu;}void LaneBorder::calcPrepareData(){LaneBorder*Gv01F=reinterpret_cast<
LaneBorder*>(getRight());LaneSection*sec=reinterpret_cast<LaneSection*>(
getParent()->getParent());mOffsetEnd=Gv01F?Gv01F->mOffset:sec->mSEnd-sec->mS;
Lane*lane=reinterpret_cast<Lane*>(getParent());if(!lane){fprintf(stderr,
"\x4c\x61\x6e\x65\x42\x6f\x72\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x6c\x61\x6e\x65\x20\x62\x6f\x72\x64\x65\x72\x20\x68\x61\x73\x20\x6e\x6f\x20\x70\x61\x72\x65\x6e\x74\x20\x6c\x61\x6e\x65\x21\x21\x21" "\n"
);return;}if(lane->mId==0){mInnerLane=0;mInnerLaneBorder=0;return;}mInnerLane=
reinterpret_cast<Lane*>((lane->mId<0)?lane->getLeft():lane->getRight());while(
mInnerLane){mInnerLaneBorder=mInnerLane->getFirstBorder();while(mInnerLaneBorder
&&(mInnerLaneBorder->mOffsetEnd<mOffset))mInnerLaneBorder=reinterpret_cast<
LaneBorder*>(mInnerLaneBorder->getRight());if(mInnerLaneBorder||(mInnerLane->mId
==0))break;mInnerLane=reinterpret_cast<Lane*>((lane->mId<0)?mInnerLane->getLeft(
):mInnerLane->getRight());mInnerLaneBorder=0;}}void LaneBorder::
calcPostPrepareData(){double h3p6P=ds2width(mOffset);double msQkJ=ds2width(
mOffsetEnd);mMinWidth=h3p6P;mMaxWidth=h3p6P;mMaxWidth=msQkJ>mMaxWidth?msQkJ:
mMaxWidth;mMinWidth=msQkJ<mMinWidth?msQkJ:mMinWidth;if((fabs(mD)<1.0e-6)&&(fabs(
mC)>1.0e-6)&&(fabs(mB)>1.0e-6)){double BJBDA=-mB/(2.0*mC);if((BJBDA>0.0)&&(BJBDA
<(mOffsetEnd-mOffset))){msQkJ=ds2width(mOffset+BJBDA);mMaxWidth=msQkJ>mMaxWidth?
msQkJ:mMaxWidth;mMinWidth=msQkJ<mMinWidth?msQkJ:mMinWidth;}}LaneBorder*earcr=
mInnerLaneBorder;while(earcr&&(earcr->mOffset<mOffsetEnd)){if(earcr->mOffsetEnd
>=mOffset){if((fabs(earcr->mD)<1.0e-6)&&(fabs(earcr->mC)>1.0e-6)&&(fabs(earcr->
mB)>1.0e-6)){double BJBDA=-earcr->mB/(2.0*earcr->mC);if((BJBDA>0.0)&&(BJBDA<(
earcr->mOffsetEnd-earcr->mOffset))&&(BJBDA>mOffset)&&(BJBDA<mOffsetEnd)){msQkJ=
ds2width(earcr->mOffset+BJBDA);mMaxWidth=msQkJ>mMaxWidth?msQkJ:mMaxWidth;
mMinWidth=msQkJ<mMinWidth?msQkJ:mMinWidth;}}}earcr=reinterpret_cast<LaneBorder*>
(earcr->getRight());}}Node*LaneBorder::getCopy(bool Mupxf){Node*dzamm=new 
LaneBorder(this);if(Mupxf)deepCopy(dzamm);return dzamm;}void LaneBorder::
getMinMax(double&hAogV,double&Fc2XK){hAogV=mMinWidth;Fc2XK=mMaxWidth;}}
