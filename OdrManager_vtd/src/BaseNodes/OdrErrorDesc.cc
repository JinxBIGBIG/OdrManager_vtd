
#include "OdrErrorDesc.hh"
#include "OdrReaderXML.hh"
#include <stdlib.h>
#include <stdio.h>
namespace OpenDrive{ErrorDesc::ErrorDesc():Node(
"\x45\x72\x72\x6f\x72\x44\x65\x73\x63"){mOpcode=ODR_OPCODE_ERROR_DESC;}ErrorDesc
::ErrorDesc(ErrorDesc*Nf2Ao):Node(Nf2Ao){mXYAbsolute=Nf2Ao->mXYAbsolute;
mZAbsolute=Nf2Ao->mZAbsolute;mXYRelative=Nf2Ao->mXYRelative;mZRelative=Nf2Ao->
mZRelative;}ErrorDesc::~ErrorDesc(){}void ErrorDesc::printData()const{std::
string DO9qH("");for(unsigned int i=0;i<mLevel;++i)DO9qH+=std::string(
"\x20\x20\x20\x20");fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x58\x59\x41\x62\x73\x6f\x6c\x75\x74\x65\x3a\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,DO9qH.c_str(),mXYAbsolute);fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x5a\x41\x62\x73\x6f\x6c\x75\x74\x65\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,DO9qH.c_str(),mZAbsolute);fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x58\x59\x52\x65\x6c\x61\x74\x69\x76\x65\x3a\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,DO9qH.c_str(),mXYRelative);fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x5a\x52\x65\x6c\x61\x74\x69\x76\x65\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,DO9qH.c_str(),mZRelative);}bool ErrorDesc::read(ReaderXML*F3vnM){mXYAbsolute=
F3vnM->getDouble("\x78\x79\x41\x62\x73\x6f\x6c\x75\x74\x65");mZAbsolute=F3vnM->
getDouble("\x7a\x41\x62\x73\x6f\x6c\x75\x74\x65");mXYRelative=F3vnM->getDouble(
"\x78\x79\x52\x65\x6c\x61\x74\x69\x76\x65");mZRelative=F3vnM->getDouble(
"\x7a\x52\x65\x6c\x61\x74\x69\x76\x65");return true;}Node*ErrorDesc::getCopy(
bool Mupxf){Node*dzamm=new ErrorDesc(this);if(Mupxf)deepCopy(dzamm);return dzamm
;}}
