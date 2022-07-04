
#include "OdrJuncRef.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OdrRoadHeader.hh"
#include "OdrJuncHeader.hh"
#include <stdio.h>
namespace OpenDrive{JuncRef::JuncRef():Node("\x4a\x75\x6e\x63\x52\x65\x66"){
mOpcode=ODR_OPCODE_JUNCTION_REFERENCE;mLevel=1;mJuncHeader=0;}JuncRef::JuncRef(
JuncRef*Nf2Ao):Node(Nf2Ao){mJuncHeader=Nf2Ao->mJuncHeader;}JuncRef::JuncRef(
JuncHeader*GqNs7):Node("\x4a\x75\x6e\x63\x52\x65\x66"){mOpcode=
ODR_OPCODE_JUNCTION_REFERENCE;mLevel=1;mJuncHeader=GqNs7;}JuncRef::~JuncRef(){}
void JuncRef::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6a\x75\x6e\x63\x3a\x20\x25\x70" "\n",
mJuncHeader);}bool JuncRef::read(ReaderXML*F3vnM){mId=F3vnM->getUInt(
"\x6a\x75\x6e\x63\x74\x69\x6f\x6e");mIdAsString=F3vnM->getString(
"\x6a\x75\x6e\x63\x74\x69\x6f\x6e");return true;}Node*JuncRef::getCopy(bool 
Mupxf){Node*dzamm=new JuncRef(this);if(Mupxf)deepCopy(dzamm);return dzamm;}void 
JuncRef::calcPrepareData(){Node*wrEZC=getParent()->getParent();if(!wrEZC)return;
JuncHeader*jHdr=reinterpret_cast<JuncHeader*>(wrEZC->getChild(
ODR_OPCODE_JUNCTION_HEADER));while(jHdr){if(jHdr->mIdAsString==mIdAsString){
mJuncHeader=jHdr;return;}jHdr=reinterpret_cast<JuncHeader*>(jHdr->getRight());}}
}
