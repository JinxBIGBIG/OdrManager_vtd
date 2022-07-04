
#include "OdrCrossfall.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{Crossfall::Crossfall():Node(
"\x43\x72\x6f\x73\x73\x66\x61\x6c\x6c"){mOpcode=ODR_OPCODE_CROSSFALL;mLevel=1;}
Crossfall::Crossfall(Crossfall*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mSide=Nf2Ao->
mSide;mA=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;}Crossfall::~Crossfall
(){}void Crossfall::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x69\x64\x65\x3a\x20\x25\x64" "\n",mSide);fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x25\x2e\x38\x6c\x66" "\n",mS);fprintf(
stderr,"\x20\x20\x20\x20\x41\x3a\x20\x20\x20\x20\x25\x2e\x38\x6c\x66" "\n",mA);
fprintf(stderr,
"\x20\x20\x20\x20\x42\x3a\x20\x20\x20\x20\x25\x2e\x38\x6c\x66" "\n",mB);fprintf(
stderr,"\x20\x20\x20\x20\x43\x3a\x20\x20\x20\x20\x25\x2e\x38\x6c\x66" "\n",mC);
fprintf(stderr,
"\x20\x20\x20\x20\x44\x3a\x20\x20\x20\x20\x25\x2e\x38\x6c\x66" "\n",mD);}bool 
Crossfall::read(ReaderXML*F3vnM){std::string bFly1=F3vnM->getString(
"\x73\x69\x64\x65");if(bFly1=="\x6c\x65\x66\x74")mSide=ODR_SIDE_LEFT;else if(
bFly1=="\x72\x69\x67\x68\x74")mSide=ODR_SIDE_RIGHT;else mSide=ODR_SIDE_BOTH;mS=
F3vnM->getDouble("\x73");mA=F3vnM->getDouble("\x61");mB=F3vnM->getDouble("\x62")
;mC=F3vnM->getDouble("\x63");mD=F3vnM->getDouble("\x64");return true;}double 
Crossfall::s2xfall(const double&BJBDA){double rganP=BJBDA-mS;return mA+mB*rganP+
mC*rganP*rganP+mD*rganP*rganP*rganP;}double Crossfall::s2xfallDot(const double&
BJBDA){double rganP=BJBDA-mS;return mB+2.0*mC*rganP+3.0*mD*rganP*rganP;}Node*
Crossfall::getCopy(bool Mupxf){Node*dzamm=new Crossfall(this);if(Mupxf)deepCopy(
dzamm);return dzamm;}}
