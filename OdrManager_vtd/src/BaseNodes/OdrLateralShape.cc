
#include "OdrLateralShape.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LateralShape::LateralShape():Node(
"\x4c\x61\x74\x65\x72\x61\x6c\x53\x68\x61\x70\x65"){mOpcode=
ODR_OPCODE_LATERAL_SHAPE;mLevel=1;}LateralShape::LateralShape(LateralShape*Nf2Ao
):Node(Nf2Ao){mS=Nf2Ao->mS;mSEnd=Nf2Ao->mSEnd;mT=Nf2Ao->mT;mTEnd=Nf2Ao->mTEnd;mA
=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;}LateralShape::~LateralShape()
{}void LateralShape::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x25\x2e\x38\x6c\x66\x20\x2d\x20\x25\x2e\x38\x6c\x66" "\n"
,mS,mSEnd);fprintf(stderr,
"\x20\x20\x20\x20\x74\x3a\x20\x25\x2e\x38\x6c\x66\x20\x2d\x20\x25\x2e\x38\x6c\x66" "\n"
,mT,mTEnd);fprintf(stderr,
"\x20\x20\x20\x20\x41\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mA);fprintf(stderr,
"\x20\x20\x20\x20\x42\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mB);fprintf(stderr,
"\x20\x20\x20\x20\x43\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mC);fprintf(stderr,
"\x20\x20\x20\x20\x44\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mD);}bool LateralShape::
read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mT=F3vnM->getDouble("\x74");
mTEnd=mT;mA=F3vnM->getDouble("\x61");mB=F3vnM->getDouble("\x62");mC=F3vnM->
getDouble("\x63");mD=F3vnM->getDouble("\x64");mSEnd=0.0;RoadHeader*hdr=
reinterpret_cast<RoadHeader*>(getParent());if(hdr)mSEnd=hdr->mLength;
LateralShape*pc23j=reinterpret_cast<LateralShape*>(getLeft());while(pc23j){if(
pc23j->mSEnd>mS)pc23j->mSEnd=mS;pc23j=reinterpret_cast<LateralShape*>(pc23j->
getLeft());}pc23j=reinterpret_cast<LateralShape*>(getLeft());if(pc23j){if(pc23j
->mS==mS)pc23j->mTEnd=mT;}return true;}double LateralShape::st2z(const double&
BJBDA,const double&rkiXc){double rganP=BJBDA-mS;double ntV87=rkiXc-mT;double 
zg1KL=mA+mB*ntV87+mC*ntV87*ntV87+mD*ntV87*ntV87*ntV87;if(rganP>0.0){LateralShape
*H3SgW=reinterpret_cast<LateralShape*>(getRight());LateralShape*ETX5D=0;while(
H3SgW){if(H3SgW->mS==mSEnd){if(H3SgW->mT<=mT)ETX5D=H3SgW;else break;}else if(
H3SgW->mS>mSEnd)break;H3SgW=reinterpret_cast<LateralShape*>(H3SgW->getRight());}
if(ETX5D){double gfGFC=ETX5D->st2z(ETX5D->mS,rkiXc);zg1KL+=(gfGFC-zg1KL)*rganP/(
mSEnd-mS);}}return zg1KL;}Node*LateralShape::getCopy(bool Mupxf){Node*dzamm=new 
LateralShape(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
