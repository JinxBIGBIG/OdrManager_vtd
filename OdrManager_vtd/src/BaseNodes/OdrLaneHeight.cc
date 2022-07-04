
#include "OdrLaneHeight.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneHeight::LaneHeight():Node(
"\x4c\x61\x6e\x65\x48\x65\x69\x67\x68\x74"){mOpcode=ODR_OPCODE_LANE_HEIGHT;
mLevel=3;mS=0.0;mSEnd=0.0;}LaneHeight::LaneHeight(LaneHeight*Nf2Ao):Node(Nf2Ao){
mOffset=Nf2Ao->mOffset;mInner=Nf2Ao->mInner;mOuter=Nf2Ao->mOuter;mS=Nf2Ao->mS;
mSEnd=Nf2Ao->mSEnd;}LaneHeight::~LaneHeight(){}void LaneHeight::printData()const
{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOffset);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x69\x6e\x6e\x65\x72\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mInner);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6f\x75\x74\x65\x72\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOuter);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x45\x6e\x64\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mSEnd);}bool LaneHeight::read(ReaderXML*F3vnM){mOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");if(F3vnM->hasAttribute(
"\x68\x65\x69\x67\x68\x74\x49\x6e\x6e\x65\x72"))mInner=F3vnM->getDouble(
"\x68\x65\x69\x67\x68\x74\x49\x6e\x6e\x65\x72");else mInner=F3vnM->getDouble(
"\x69\x6e\x6e\x65\x72");if(F3vnM->hasAttribute(
"\x68\x65\x69\x67\x68\x74\x4f\x75\x74\x65\x72"))mOuter=F3vnM->getDouble(
"\x68\x65\x69\x67\x68\x74\x4f\x75\x74\x65\x72");else mOuter=F3vnM->getDouble(
"\x6f\x75\x74\x65\x72");return true;}double LaneHeight::dsdt2height(const double
&rganP,const double&ntV87){double BJBDA=rganP-mOffset;if(BJBDA<0.0||ntV87<0.0)
return 0.0;return mInner;}double LaneHeight::sAndRelOffset2height(const double&
BJBDA,const double&ZwrVB){double N8AeH=0.0;if((mSEnd-mS)>1.0e-5)N8AeH=(BJBDA-mS)
/(mSEnd-mS);double o_hu2=mInner;double o5ghr=mOuter;LaneHeight*N2pqv=
reinterpret_cast<LaneHeight*>(getRight());if(N2pqv){o_hu2=mInner+N8AeH*(N2pqv->
mInner-mInner);o5ghr=mOuter+N8AeH*(N2pqv->mOuter-mOuter);}return o_hu2+ZwrVB*(
o5ghr-o_hu2);}Node*LaneHeight::getCopy(bool Mupxf){Node*dzamm=new LaneHeight(
this);if(Mupxf)deepCopy(dzamm);return dzamm;}void LaneHeight::calcPrepareData(){
LaneSection*H4prs=reinterpret_cast<LaneSection*>(getParent()->getParent());mS=
H4prs->mS+mOffset;mSEnd=H4prs->mSEnd;LaneHeight*ZrU8W=reinterpret_cast<
LaneHeight*>(getLeft());if(ZrU8W)ZrU8W->mSEnd=mS;}}
