
#include "OdrCornerReference.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{CornerReference::CornerReference():Node(
"\x43\x6f\x72\x6e\x65\x72\x52\x65\x66\x65\x72\x65\x6e\x63\x65"){mOpcode=
ODR_OPCODE_CORNER_REFERENCE;mLevel=4;}CornerReference::CornerReference(
CornerReference*Nf2Ao):Node(Nf2Ao){mId=Nf2Ao->mId;}CornerReference::~
CornerReference(){}void CornerReference::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x69\x64\x3a\x20\x25\x2e\x64" "\n",mId);}bool 
CornerReference::read(ReaderXML*F3vnM){mId=F3vnM->getInt("\x69\x64");return true
;}Node*CornerReference::getCopy(bool Mupxf){Node*dzamm=new CornerReference(this)
;if(Mupxf)deepCopy(dzamm);return dzamm;}}
