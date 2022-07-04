
#include "OdrCornerLocal.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{CornerLocal::CornerLocal():Node(
"\x43\x6f\x72\x6e\x65\x72\x4c\x6f\x63\x61\x6c"){mOpcode=ODR_OPCODE_CORNER_LOCAL;
mLevel=4;}CornerLocal::CornerLocal(CornerLocal*Nf2Ao):Node(Nf2Ao){mU=Nf2Ao->mU;
mV=Nf2Ao->mV;mZ=Nf2Ao->mZ;mHeight=Nf2Ao->mHeight;mId=Nf2Ao->mId;}CornerLocal::~
CornerLocal(){}void CornerLocal::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x64" "\n",
mId);fprintf(stderr,
"\x20\x20\x20\x20\x75\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mU);fprintf(stderr,
"\x20\x20\x20\x20\x76\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mV);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZ);fprintf(stderr,
"\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHeight);}bool CornerLocal::read(ReaderXML*F3vnM){mId=F3vnM->getInt("\x69\x64")
;mU=F3vnM->getDouble("\x75");mV=F3vnM->getDouble("\x76");mZ=F3vnM->getDouble(
"\x7a");mHeight=F3vnM->getDouble("\x68\x65\x69\x67\x68\x74");return true;}Node*
CornerLocal::getCopy(bool Mupxf){Node*dzamm=new CornerLocal(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}}
