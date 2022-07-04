
#include "OdrUserData.hh"
#include "OdrGenericNode.hh"
#include "OdrRailroadSwitch.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
namespace OpenDrive{UserData::UserData():Node("\x55\x73\x65\x72\x44\x61\x74\x61"
),mData(0),mSize(0),mCode(""),mValue(""){mOpcode=ODR_OPCODE_USER_DATA;}UserData
::UserData(UserData*Nf2Ao):Node(Nf2Ao){mSize=Nf2Ao->mSize;mCode=Nf2Ao->mCode;
mValue=Nf2Ao->mValue;mData=calloc(mSize,1);memcpy(mData,Nf2Ao->mData,mSize);}
UserData::~UserData(){if(mData)free(mData);}void UserData::printData()const{std
::string DO9qH("");for(unsigned int i=0;i<mLevel;++i)DO9qH+=std::string(
"\x20\x20\x20\x20");fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x43\x6f\x64\x65\x3a\x20\x20\x20\x25\x73" "\n",
DO9qH.c_str(),mCode.c_str());fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x6d\x56\x61\x6c\x75\x65\x3a\x20\x20\x25\x73" "\n",
DO9qH.c_str(),mValue.c_str());}bool UserData::read(ReaderXML*F3vnM){mCode=F3vnM
->getString("\x63\x6f\x64\x65");mValue=F3vnM->getString("\x76\x61\x6c\x75\x65");
return true;}Node*UserData::getCopy(bool Mupxf){Node*dzamm=new UserData(this);if
(Mupxf)deepCopy(dzamm);return dzamm;}void UserData::calcPrepareData(){if(mCode!=
"\x76\x69\x53\x77\x69\x74\x63\x68\x4c\x69\x73\x74")return;OpenDrive::GenericNode
*CEUl4=reinterpret_cast<OpenDrive::GenericNode*>(getChild());while(CEUl4){if(
CEUl4->getName()=="\x73\x77\x69\x74\x63\x68")getParent()->addChild(new 
RailroadSwitch(CEUl4));CEUl4=reinterpret_cast<OpenDrive::GenericNode*>(CEUl4->
getRight());}}}
