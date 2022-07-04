
#include "OdrOffset.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{Offset::Offset():Node("\x4f\x66\x66\x73\x65\x74"){mOpcode=
ODR_OPCODE_OFFSET;mLevel=0;}Offset::Offset(Offset*Nf2Ao):Node(Nf2Ao){mX=Nf2Ao->
mX;mY=Nf2Ao->mY;mZ=Nf2Ao->mZ;mH=Nf2Ao->mH;}Offset::~Offset(){}void Offset::
printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x78\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mX);fprintf(stderr,
"\x20\x20\x20\x20\x79\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mY);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZ);fprintf(stderr,
"\x20\x20\x20\x20\x68\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mH);}bool Offset::read(ReaderXML*F3vnM){mX=F3vnM->getDouble("\x78");mY=F3vnM->
getDouble("\x79");mZ=F3vnM->getDouble("\x7a");mH=F3vnM->getDouble("\x68\x64\x67"
);return true;}Node*Offset::getCopy(bool Mupxf){Node*dzamm=new Offset(this);if(
Mupxf)deepCopy(dzamm);return dzamm;}}
