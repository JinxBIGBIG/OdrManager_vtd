
#include "OdrReference.hh"
#include "OdrObject.hh"
#include "OdrSignal.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{Reference::Reference():Node(
"\x52\x65\x66\x65\x72\x65\x6e\x63\x65"){mOpcode=ODR_OPCODE_REFERENCE;mLevel=2;}
Reference::Reference(Reference*Nf2Ao):Node(Nf2Ao){mElementType=Nf2Ao->
mElementType;mElementId=Nf2Ao->mElementId;mType=Nf2Ao->mType;mRefObject=Nf2Ao->
mRefObject;mRefSignal=Nf2Ao->mRefSignal;}Reference::~Reference(){}void Reference
::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x45\x6c\x65\x6d\x65\x6e\x74\x54\x79\x70\x65\x3a\x20\x25\x73" "\n"
,mElementType.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x45\x6c\x65\x6d\x65\x6e\x74\x49\x64\x3a\x20\x20\x20\x25\x73" "\n"
,mElementId.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mType.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x52\x65\x66\x4f\x62\x6a\x65\x63\x74\x3a\x20\x20\x20\x25\x70" "\n"
,mRefObject);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x52\x65\x66\x53\x69\x67\x6e\x61\x6c\x3a\x20\x20\x20\x25\x70" "\n"
,mRefSignal);}bool Reference::read(ReaderXML*F3vnM){mElementType=F3vnM->
getString("\x65\x6c\x65\x6d\x65\x6e\x74\x54\x79\x70\x65");mElementId=F3vnM->
getString("\x65\x6c\x65\x6d\x65\x6e\x74\x49\x64");mType=F3vnM->getString(
"\x74\x79\x70\x65");mRefObject=0;mRefSignal=0;return true;}Node*Reference::
getCopy(bool Mupxf){Node*dzamm=new Reference(this);if(Mupxf)deepCopy(dzamm);
return dzamm;}void Reference::calcPrepareData(){bool ce6FN=mElementType==std::
string("\x6f\x62\x6a\x65\x63\x74");bool MY61a=mElementType==std::string(
"\x73\x69\x67\x6e\x61\x6c");if(!ce6FN&&!MY61a)return;RoadHeader*REKut=
reinterpret_cast<RoadHeader*>(getParent()->getParent()->getParent()->getChild(
ODR_OPCODE_ROAD_HEADER));while(REKut&&!mRefObject&&!mRefSignal){if(MY61a){Signal
*PERPi=reinterpret_cast<Signal*>(REKut->getChild(ODR_OPCODE_SIGNAL));while(PERPi
&&!mRefSignal){if(PERPi->mIdAsString==mElementId)mRefSignal=PERPi;PERPi=
reinterpret_cast<Signal*>(PERPi->getRight());}}else if(ce6FN){Object*an8Kh=
reinterpret_cast<Object*>(REKut->getChild(ODR_OPCODE_OBJECT));while(an8Kh&&!
mRefObject){if(an8Kh->mIdAsString==mElementId)mRefObject=an8Kh;an8Kh=
reinterpret_cast<Object*>(an8Kh->getRight());}}REKut=reinterpret_cast<RoadHeader
*>(REKut->getRight());}}}
