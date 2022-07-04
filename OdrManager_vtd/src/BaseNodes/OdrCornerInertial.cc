
#include "OdrCornerInertial.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{CornerInertial::CornerInertial():Node(
"\x43\x6f\x72\x6e\x65\x72\x49\x6e\x65\x72\x74\x69\x61\x6c"){mOpcode=
ODR_OPCODE_CORNER_INERTIAL;mLevel=4;}CornerInertial::CornerInertial(
CornerInertial*Nf2Ao):Node(Nf2Ao){mX=Nf2Ao->mX;mY=Nf2Ao->mY;mZ=Nf2Ao->mZ;mHeight
=Nf2Ao->mHeight;mId=Nf2Ao->mId;}CornerInertial::~CornerInertial(){}void 
CornerInertial::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x64" "\n",
mId);fprintf(stderr,
"\x20\x20\x20\x20\x78\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mX);fprintf(stderr,
"\x20\x20\x20\x20\x79\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mY);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZ);fprintf(stderr,
"\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHeight);}bool CornerInertial::read(ReaderXML*F3vnM){mId=F3vnM->getInt(
"\x69\x64");mX=F3vnM->getDouble("\x78");mY=F3vnM->getDouble("\x79");mZ=F3vnM->
getDouble("\x7a");mHeight=F3vnM->getDouble("\x68\x65\x69\x67\x68\x74");return 
true;}Node*CornerInertial::getCopy(bool Mupxf){Node*dzamm=new CornerInertial(
this);if(Mupxf)deepCopy(dzamm);return dzamm;}void CornerInertial::
applyTransformation(const double&dx,const double&dy,const double&Zbgoe,const 
double&nZWMW){mX+=dx;mY+=dy;mZ+=Zbgoe;rotatePoint(mX,mY,0.0,0.0,nZWMW);}}
