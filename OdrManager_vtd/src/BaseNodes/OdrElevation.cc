
#include "OdrElevation.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{Elevation::Elevation():Node(
"\x45\x6c\x65\x76\x61\x74\x69\x6f\x6e"){mOpcode=ODR_OPCODE_ELEVATION;mLevel=1;}
Elevation::Elevation(Elevation*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mSEnd=Nf2Ao->
mSEnd;mA=Nf2Ao->mA;mB=Nf2Ao->mB;mC=Nf2Ao->mC;mD=Nf2Ao->mD;}Elevation::~Elevation
(){}void Elevation::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x25\x2e\x38\x6c\x66\x20\x2d\x20\x25\x2e\x38\x6c\x66" "\n"
,mS,mSEnd);fprintf(stderr,
"\x20\x20\x20\x20\x41\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mA);fprintf(stderr,
"\x20\x20\x20\x20\x42\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mB);fprintf(stderr,
"\x20\x20\x20\x20\x43\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mC);fprintf(stderr,
"\x20\x20\x20\x20\x44\x3a\x20\x25\x2e\x38\x6c\x66" "\n",mD);}bool Elevation::
read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mA=F3vnM->getDouble("\x61");mB
=F3vnM->getDouble("\x62");mC=F3vnM->getDouble("\x63");mD=F3vnM->getDouble("\x64"
);mSEnd=0.0;RoadHeader*hdr=reinterpret_cast<RoadHeader*>(getParent());if(hdr)
mSEnd=hdr->mLength;Elevation*Thkia=reinterpret_cast<Elevation*>(getLeft());if(
Thkia)Thkia->mSEnd=mS;return true;}double Elevation::s2z(const double&BJBDA){
double rganP=BJBDA-mS;return mA+mB*rganP+mC*rganP*rganP+mD*rganP*rganP*rganP;}
double Elevation::s2pitch(const double&BJBDA){double rganP=BJBDA-mS;return-1.0*
atan(mB+2.0*mC*rganP+3.0*mD*rganP*rganP);}double Elevation::s2pitchDot(const 
double&BJBDA){double rganP=BJBDA-mS;double es9sx=mB+2.0*mC*rganP+3.0*mD*rganP*
rganP;return-1.0*(2.0*mC+6.0*mD*rganP)/(1.0+es9sx*es9sx);}double Elevation::
s2secondDeriv(const double&BJBDA){double rganP=BJBDA-mS;return 2.0*mC+6.0*mD*
rganP;}Node*Elevation::getCopy(bool Mupxf){Node*dzamm=new Elevation(this);if(
Mupxf)deepCopy(dzamm);return dzamm;}double Elevation::s2curvature(const double&
BJBDA){double rganP=BJBDA-mS;double lgWm4=mB+2.0*mC*rganP+3.0*mD*rganP*rganP;
double sj8U1=2.0*mC+6.0*mD*rganP;double value=1+lgWm4*lgWm4;value=sqrt(value*
value*value);return sj8U1/value;}void Elevation::applyTransformation(const 
double&,const double&,const double&Zbgoe,const double&){mA+=Zbgoe;}}
