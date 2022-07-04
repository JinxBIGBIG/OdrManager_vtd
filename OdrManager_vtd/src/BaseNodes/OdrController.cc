
#include "OdrController.hh"
#include "OdrJuncController.hh"
#include "OdrControlEntry.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{Controller::Controller():Node(
"\x43\x6f\x6e\x74\x72\x6f\x6c\x6c\x65\x72"){mOpcode=ODR_OPCODE_CONTROLLER;}
Controller::~Controller(){}void Controller::printData()const{fprintf(stderr,
"\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x73" "\n",mName.c_str());
fprintf(stderr,
"\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n",
mIdAsString.c_str(),mId);}bool Controller::read(ReaderXML*F3vnM){mName=F3vnM->
getString("\x6e\x61\x6d\x65");mId=F3vnM->getUInt("\x69\x64");mIdAsString=F3vnM->
getString("\x69\x64");return true;}ControlEntry*Controller::getFirstEntry(){
return reinterpret_cast<ControlEntry*>(getChild(ODR_OPCODE_CONTROL_ENTRY));}void
 Controller::calcPrepareData(){Node*node=getParent()->getChild(
ODR_OPCODE_JUNCTION_HEADER);while(node&&(node->getOpcode()==
ODR_OPCODE_JUNCTION_HEADER)){JuncController*xpKIZ=reinterpret_cast<
JuncController*>(node->getChild(ODR_OPCODE_JUNCTION_CONTROL));while(xpKIZ){if(
xpKIZ->mIdAsString==mIdAsString)xpKIZ->setController(this);xpKIZ=
reinterpret_cast<JuncController*>(xpKIZ->getRight());}node=node->getRight();}}}
