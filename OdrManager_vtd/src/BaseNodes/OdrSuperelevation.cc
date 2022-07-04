
#include "OdrSuperelevation.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{Superelevation::Superelevation():Node(
"\x53\x75\x70\x65\x72\x65\x6c\x65\x76\x61\x74\x69\x6f\x6e"){mOpcode=
ODR_OPCODE_SUPERELEVATION;mLevel=1;}Superelevation::Superelevation(
Superelevation*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mSEnd=Nf2Ao->mSEnd;mA=Nf2Ao->mA;
mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;}Superelevation::~Superelevation(){}void 
Superelevation::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x25\x2e\x38\x6c\x66\x20\x2d\x20\x25\x2e\x38\x6c\x66" "\n"
,mS,mSEnd);fprintf(stderr,
"\x20\x20\x20\x20\x41\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mA);fprintf(stderr,
"\x20\x20\x20\x20\x42\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mB);fprintf(stderr,
"\x20\x20\x20\x20\x43\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mC);fprintf(stderr,
"\x20\x20\x20\x20\x44\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mD);}bool Superelevation
::read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mA=F3vnM->getDouble("\x61");
mB=F3vnM->getDouble("\x62");mC=F3vnM->getDouble("\x63");mD=F3vnM->getDouble(
"\x64");mSEnd=0.0;RoadHeader*hdr=reinterpret_cast<RoadHeader*>(getParent());if(
hdr)mSEnd=hdr->mLength;Superelevation*lsnB1=reinterpret_cast<Superelevation*>(
getLeft());if(lsnB1)lsnB1->mSEnd=mS;return true;}double Superelevation::s2xfall(
const double&BJBDA){double rganP=BJBDA-mS;return mA+mB*rganP+mC*rganP*rganP+mD*
rganP*rganP*rganP;}double Superelevation::s2xfallDot(const double&BJBDA){double 
rganP=BJBDA-mS;return mB+2.0*mC*rganP+3.0*mD*rganP*rganP;}Node*Superelevation::
getCopy(bool Mupxf){Node*dzamm=new Superelevation(this);if(Mupxf)deepCopy(dzamm)
;return dzamm;}}
