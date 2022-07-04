
#include "OdrSignalDep.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{SignalDep::SignalDep():Node(
"\x53\x69\x67\x6e\x61\x6c\x44\x65\x70"){mOpcode=ODR_OPCODE_SIGNAL_DEPEND;mLevel=
2;}SignalDep::SignalDep(SignalDep*Nf2Ao):Node(Nf2Ao){mId=Nf2Ao->mId;mIdAsString=
Nf2Ao->mIdAsString;mType=Nf2Ao->mType;}SignalDep::~SignalDep(){}void SignalDep::
printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x49\x44\x3a\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x25\x64" "\n",mType);}
bool SignalDep::read(ReaderXML*F3vnM){mId=F3vnM->getUInt("\x69\x64");mIdAsString
=F3vnM->getString("\x69\x64");mType=F3vnM->getUInt("\x74\x79\x70\x65");return 
true;}Node*SignalDep::getCopy(bool Mupxf){Node*dzamm=new SignalDep(this);if(
Mupxf)deepCopy(dzamm);return dzamm;}}
