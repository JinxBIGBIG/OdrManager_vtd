
#include "OdrCornerRelative.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{CornerRelative::CornerRelative():Node(
"\x43\x6f\x72\x6e\x65\x72\x52\x65\x6c\x61\x74\x69\x76\x65"){mOpcode=
ODR_OPCODE_CORNER_RELATIVE;mLevel=4;}CornerRelative::CornerRelative(
CornerRelative*Nf2Ao):Node(Nf2Ao){mU=Nf2Ao->mU;mV=Nf2Ao->mV;mZ=Nf2Ao->mZ;mHeight
=Nf2Ao->mHeight;mId=Nf2Ao->mId;}CornerRelative::~CornerRelative(){}void 
CornerRelative::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x64" "\n",
mId);fprintf(stderr,
"\x20\x20\x20\x20\x75\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mU);fprintf(stderr,
"\x20\x20\x20\x20\x76\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mV);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZ);fprintf(stderr,
"\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHeight);}bool CornerRelative::read(ReaderXML*F3vnM){mId=F3vnM->getInt(
"\x69\x64");mU=F3vnM->getDouble("\x75");mV=F3vnM->getDouble("\x76");mZ=F3vnM->
getDouble("\x7a");mHeight=F3vnM->getDouble("\x68\x65\x69\x67\x68\x74");return 
true;}Node*CornerRelative::getCopy(bool Mupxf){Node*dzamm=new CornerRelative(
this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
