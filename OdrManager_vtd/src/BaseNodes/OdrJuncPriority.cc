
#include "OdrJuncPriority.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{JuncPriority::JuncPriority():Node(
"\x4a\x75\x6e\x63\x50\x72\x69\x6f\x72\x69\x74\x79"){mOpcode=
ODR_OPCODE_JUNCTION_PRIORITY;mLevel=1;}JuncPriority::~JuncPriority(){}void 
JuncPriority::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x50\x72\x69\x6f\x72\x69\x74\x69\x7a\x65\x64\x20\x50\x61\x74\x68\x3a\x20\x25\x64" "\n"
,mPrioId);fprintf(stderr,
"\x20\x20\x20\x20\x49\x6e\x66\x65\x72\x69\x6f\x72\x20\x50\x61\x74\x68\x3a\x20\x20\x20\x20\x25\x64" "\n"
,mLowPath);}bool JuncPriority::read(ReaderXML*F3vnM){mPrioId=F3vnM->getUInt(
"\x68\x69\x67\x68");mLowPath=F3vnM->getUInt("\x6c\x6f\x77");return true;}}
