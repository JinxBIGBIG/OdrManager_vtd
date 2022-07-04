
#include "OdrGenericNode.hh"
#include "OdrRailroadSwitch.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadHeader.hh"
#include <stdio.h>
#include <math.h>
#include <cstdlib>
namespace OpenDrive{RailroadSwitch::RailroadSwitch():Node(
"\x52\x61\x69\x6c\x72\x6f\x61\x64\x53\x77\x69\x74\x63\x68"){mOpcode=
ODR_OPCODE_RAILROAD_SWITCH;mLevel=1;mIsDynamic=true;mReadMainTrack=false;
mReadSideTrack=false;mState=ODR_RAILROAD_SWITCH_STATE_STRAIGHT;}RailroadSwitch::
RailroadSwitch(RailroadSwitch*Nf2Ao):Node(Nf2Ao){mS=Nf2Ao->mS;mId=Nf2Ao->mId;
mIdAsString=Nf2Ao->mIdAsString;mName=Nf2Ao->mName;mSideTrackId=Nf2Ao->
mSideTrackId;mSideTrackIdAsString=Nf2Ao->mSideTrackIdAsString;mSideTrackS=Nf2Ao
->mSideTrackS;mSideTrackDir=Nf2Ao->mSideTrackDir;mState=Nf2Ao->mState;mDir=Nf2Ao
->mDir;}RailroadSwitch::RailroadSwitch(GenericNode*SkShA):Node(
"\x52\x61\x69\x6c\x72\x6f\x61\x64\x53\x77\x69\x74\x63\x68"){mOpcode=
ODR_OPCODE_RAILROAD_SWITCH;mLevel=1;mState=ODR_RAILROAD_SWITCH_STATE_STRAIGHT;if
(!SkShA)return;for(OpenDrive::ReaderXML::AttribMap::const_iterator GHusr=SkShA->
mAttributesMap.begin();GHusr!=SkShA->mAttributesMap.end();++GHusr){if(GHusr->
first=="\x69\x64"){mId=atoi(GHusr->second.c_str());mIdAsString=GHusr->second;}if
(GHusr->first=="\x6e\x61\x6d\x65")mName=GHusr->second;}OpenDrive::GenericNode*
tx9D4=reinterpret_cast<OpenDrive::GenericNode*>(SkShA->getChild());while(tx9D4){
if(tx9D4->getName()=="\x6d\x61\x69\x6e\x54\x72\x61\x63\x6b"){for(OpenDrive::
ReaderXML::AttribMap::const_iterator GHusr=tx9D4->mAttributesMap.begin();GHusr!=
tx9D4->mAttributesMap.end();++GHusr){if(GHusr->first=="\x73")mS=atof(GHusr->
second.c_str());if(GHusr->first=="\x64\x69\x72")mDir=(GHusr->second=="\x2b")?
ODR_DIRECTION_PLUS:ODR_DIRECTION_MINUS;}}if(tx9D4->getName()==
"\x73\x69\x64\x65\x54\x72\x61\x63\x6b"){for(OpenDrive::ReaderXML::AttribMap::
const_iterator GHusr=tx9D4->mAttributesMap.begin();GHusr!=tx9D4->mAttributesMap.
end();++GHusr){if(GHusr->first=="\x73")mSideTrackS=atof(GHusr->second.c_str());
if(GHusr->first=="\x64\x69\x72")mSideTrackDir=(GHusr->second=="\x2b")?
ODR_DIRECTION_PLUS:ODR_DIRECTION_MINUS;if(GHusr->first=="\x69\x64"){mSideTrackId
=atoi(GHusr->second.c_str());mSideTrackIdAsString=GHusr->second;}}}tx9D4=
reinterpret_cast<OpenDrive::GenericNode*>(tx9D4->getRight());}}RailroadSwitch::~
RailroadSwitch(){}void RailroadSwitch::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x69\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mName.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x73\x69\x64\x65\x54\x72\x61\x63\x6b\x3a\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mSideTrackIdAsString.c_str(),mSideTrackId);fprintf(stderr,
"\x20\x20\x20\x20\x73\x69\x64\x65\x54\x72\x61\x63\x6b\x53\x3a\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mSideTrackS);fprintf(stderr,
"\x20\x20\x20\x20\x73\x69\x64\x65\x54\x72\x61\x63\x6b\x44\x69\x72\x3a\x20\x25\x64" "\n"
,mSideTrackDir);fprintf(stderr,
"\x20\x20\x20\x20\x73\x74\x61\x74\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mState);fprintf(stderr,
"\x20\x20\x20\x20\x64\x69\x72\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mDir);}bool RailroadSwitch::read(ReaderXML*F3vnM){if(mReadMainTrack){
mReadMainTrack=false;RoadHeader*M8Gux=reinterpret_cast<RoadHeader*>(getParent())
;if(M8Gux){if(M8Gux->mIdAsString!=F3vnM->getString("\x69\x64")){fprintf(stderr,
"\x52\x61\x69\x6c\x72\x6f\x61\x64\x53\x77\x69\x74\x63\x68\x3a\x3a\x72\x65\x61\x64\x3a\x20\x57\x41\x52\x4e\x49\x4e\x47\x3a\x20\x73\x77\x69\x74\x63\x68\x20\x69\x64\x20\x3c\x25\x73\x3e\x3a\x20\x6d\x61\x69\x6e\x20\x54\x72\x61\x63\x6b\x20\x69\x64\x20\x3c\x25\x73\x3e\x20\x69\x73\x20\x64\x69\x66\x66\x65\x72\x65\x6e\x74\x20\x66\x72\x6f\x6d\x20\x70\x61\x72\x65\x6e\x74\x20\x69\x64\x20\x3c\x25\x73\x3e\x2e\x20\x54\x68\x69\x73\x20\x6d\x75\x73\x74\x20\x6e\x6f\x74\x20\x68\x61\x70\x70\x65\x6e\x21" "\n"
,mIdAsString.c_str(),F3vnM->getString("\x69\x64").c_str(),M8Gux->mIdAsString.
c_str());return false;}}mS=F3vnM->getDouble("\x73");mDir=(F3vnM->getString(
"\x64\x69\x72")=="\x2b")?ODR_DIRECTION_PLUS:((F3vnM->getString("\x64\x69\x72")==
"\x2d"?ODR_DIRECTION_MINUS:ODR_DIRECTION_NONE));}else if(mReadSideTrack){
mSideTrackId=F3vnM->getUInt("\x69\x64");mSideTrackIdAsString=F3vnM->getString(
"\x69\x64");mSideTrackS=F3vnM->getDouble("\x73");mSideTrackDir=(F3vnM->getString
("\x64\x69\x72")=="\x2b")?ODR_DIRECTION_PLUS:((F3vnM->getString("\x64\x69\x72")
=="\x2d"?ODR_DIRECTION_MINUS:ODR_DIRECTION_NONE));mReadSideTrack=false;}else{mId
=F3vnM->getUInt("\x69\x64");mIdAsString=F3vnM->getString("\x69\x64");mName=F3vnM
->getString("\x6e\x61\x6d\x65");mIsDynamic=F3vnM->getString(
"\x70\x6f\x73\x69\x74\x69\x6f\x6e")=="\x64\x79\x6e\x61\x6d\x69\x63";mState=
ODR_RAILROAD_SWITCH_STATE_STRAIGHT;if(!mIsDynamic&&(F3vnM->getString(
"\x70\x6f\x73\x69\x74\x69\x6f\x6e")=="\x74\x75\x72\x6e"))mState=
ODR_RAILROAD_SWITCH_STATE_TURN;}return true;}Node*RailroadSwitch::getCopy(bool 
Mupxf){Node*dzamm=new RailroadSwitch(this);if(Mupxf)deepCopy(dzamm);return dzamm
;}void RailroadSwitch::readMainTrack(){mReadMainTrack=true;}void RailroadSwitch
::readSideTrack(){mReadSideTrack=true;}}
