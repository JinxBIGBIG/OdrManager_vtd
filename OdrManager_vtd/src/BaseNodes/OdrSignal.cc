
#include "OdrSignal.hh"
#include "OdrRoadHeader.hh"
#include "OdrGeoHeader.hh"
#include "OdrGeoNode.hh"
#include "OdrElevation.hh"
#include "OdrReaderXML.hh"
#include "OdrController.hh"
#include "OdrControlEntry.hh"
#include "OdrUserData.hh"
#include "OdrLaneValidity.hh"
#include "OdrObject.hh" 
#include "OdrRoadData.hh" 
#include <stdio.h>
#include <math.h>
namespace OpenDrive{Signal::Signal():Node("\x53\x69\x67\x6e\x61\x6c"),mInX(0.0),
mInY(0.0),mInH(0.0),mController(0),mControlEntry(0){mOpcode=ODR_OPCODE_SIGNAL;
mLevel=1;mLinkedObject=0;mLinkedObjectId="";}Signal::Signal(Signal*Nf2Ao):Node(
Nf2Ao){mS=Nf2Ao->mS;mT=Nf2Ao->mT;mId=Nf2Ao->mId;mIdAsString=Nf2Ao->mIdAsString;
mName=Nf2Ao->mName;mDynamic=Nf2Ao->mDynamic;mDir=Nf2Ao->mDir;mZ=Nf2Ao->mZ;
mCountry=Nf2Ao->mCountry;mCountryRevision=Nf2Ao->mCountryRevision;mType=Nf2Ao->
mType;mTypeAsString=Nf2Ao->mTypeAsString;mSubType=Nf2Ao->mSubType;
mSubTypeAsString=Nf2Ao->mSubTypeAsString;mValue=Nf2Ao->mValue;mInX=Nf2Ao->mInX;
mInY=Nf2Ao->mInY;mInZ=Nf2Ao->mInZ;mInH=Nf2Ao->mInH;mController=Nf2Ao->
mController;mControlEntry=Nf2Ao->mControlEntry;mState=Nf2Ao->mState;mReadability
=Nf2Ao->mReadability;mOcclusion=Nf2Ao->mOcclusion;mMinLane=Nf2Ao->mMinLane;
mMaxLane=Nf2Ao->mMaxLane;mUnit=Nf2Ao->mUnit;mHeight=Nf2Ao->mHeight;mWidth=Nf2Ao
->mWidth;mText=Nf2Ao->mText;mOffsetHdg=Nf2Ao->mOffsetHdg;mPitch=Nf2Ao->mPitch;
mRoll=Nf2Ao->mRoll;mLinkedObject=Nf2Ao->mLinkedObject;}Signal::~Signal(){}void 
Signal::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mT);fprintf(stderr,
"\x20\x20\x20\x20\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mIdAsString.c_str(),mId);fprintf(stderr,
"\x20\x20\x20\x20\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mName.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x44\x79\x6e\x61\x6d\x69\x63\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mDynamic);fprintf(stderr,
"\x20\x20\x20\x20\x64\x69\x72\x65\x63\x74\x69\x6f\x6e\x3a\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mDir);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZ);fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x75\x6e\x74\x72\x79\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e" "\n"
,mCountry.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x75\x6e\x74\x72\x79\x52\x65\x76\x69\x73\x69\x6f\x6e\x3a\x20\x3c\x25\x73\x3e" "\n"
,mCountryRevision.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mTypeAsString.c_str(),mType);fprintf(stderr,
"\x20\x20\x20\x20\x53\x75\x62\x74\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n"
,mSubTypeAsString.c_str(),mSubType);fprintf(stderr,
"\x20\x20\x20\x20\x56\x61\x6c\x75\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mValue);fprintf(stderr,
"\x20\x20\x20\x20\x73\x74\x61\x74\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mState);fprintf(stderr,
"\x20\x20\x20\x20\x75\x6e\x69\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mUnit);fprintf(stderr,
"\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHeight);fprintf(stderr,
"\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mWidth);fprintf(stderr,
"\x20\x20\x20\x20\x74\x65\x78\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mText.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x68\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mOffsetHdg);fprintf(stderr,
"\x20\x20\x20\x20\x70\x69\x74\x63\x68\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mPitch);fprintf(stderr,
"\x20\x20\x20\x20\x72\x6f\x6c\x6c\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mRoll);if(mController){fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x6e\x74\x72\x6f\x6c\x6c\x65\x72\x3a\x20\x20\x20\x30\x78\x25\x6c\x78" "\n"
,(unsigned long long)mController);fprintf(stderr,
"\x20\x20\x20\x20\x43\x6f\x6e\x74\x72\x6f\x6c\x45\x6e\x74\x72\x79\x3a\x20\x30\x78\x25\x6c\x78" "\n"
,(unsigned long long)mControlEntry);}if(mLinkedObject)fprintf(stderr,
"\x20\x20\x20\x20\x6d\x4c\x69\x6e\x6b\x65\x64\x4f\x62\x6a\x65\x63\x74\x3a\x20\x25\x70" "\n"
,mLinkedObject);}bool Signal::read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");
mT=F3vnM->getDouble("\x74");mId=F3vnM->getUInt("\x69\x64");mIdAsString=F3vnM->
getString("\x69\x64");mName=F3vnM->getString("\x6e\x61\x6d\x65");mDynamic=F3vnM
->getString("\x64\x79\x6e\x61\x6d\x69\x63")=="\x79\x65\x73";mDir=(F3vnM->
getString("\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2b")?
ODR_DIRECTION_PLUS:((F3vnM->getString(
"\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x2d"?ODR_DIRECTION_MINUS:
ODR_DIRECTION_NONE));mZ=F3vnM->getDouble("\x7a\x4f\x66\x66\x73\x65\x74");
mCountry=F3vnM->getString("\x63\x6f\x75\x6e\x74\x72\x79");mCountryRevision=F3vnM
->getString("\x63\x6f\x75\x6e\x74\x72\x79\x52\x65\x76\x69\x73\x69\x6f\x6e");
mType=F3vnM->getInt("\x74\x79\x70\x65");mTypeAsString=F3vnM->getString(
"\x74\x79\x70\x65");mSubType=F3vnM->getInt("\x73\x75\x62\x74\x79\x70\x65");
mSubTypeAsString=F3vnM->getString("\x73\x75\x62\x74\x79\x70\x65");mValue=F3vnM->
getDouble("\x76\x61\x6c\x75\x65");if((F3vnM->getString("\x76\x61\x6c\x75\x65")==
"\x6e\x6f\x20\x6c\x69\x6d\x69\x74")||(F3vnM->getString("\x76\x61\x6c\x75\x65")==
"\x75\x6e\x64\x65\x66\x69\x6e\x65\x64"))mValue=1.0e10;mUnit=F3vnM->
getOpcodeFromUnit(F3vnM->getString("\x75\x6e\x69\x74"));mHeight=F3vnM->getDouble
("\x68\x65\x69\x67\x68\x74");mWidth=F3vnM->getDouble("\x77\x69\x64\x74\x68");
mText=F3vnM->getString("\x74\x65\x78\x74");mOffsetHdg=F3vnM->getDouble(
"\x68\x4f\x66\x66\x73\x65\x74");mPitch=F3vnM->getDouble("\x70\x69\x74\x63\x68");
mRoll=F3vnM->getDouble("\x72\x6f\x6c\x6c");mState=0;mReadability=1.0;mOcclusion=
0.0;mMinLane=-99;mMaxLane=99;mLinkedObjectId=F3vnM->getString(
"\x6f\x62\x6a\x65\x63\x74\x52\x65\x66\x65\x72\x65\x6e\x63\x65");return true;}
const double&Signal::getS()const{return mS;}void Signal::calcPrepareData(){
RoadHeader*M8Gux=reinterpret_cast<RoadHeader*>(getParent());if(!M8Gux)return;
GeoHeader*RTknE=reinterpret_cast<GeoHeader*>(M8Gux->getFirstGeoHeader());if(!
RTknE)return;while(RTknE&&RTknE->getRight()&&(RTknE->mSEnd<mS))RTknE=
reinterpret_cast<GeoHeader*>(RTknE->getRight());if(!RTknE)return;GeoNode*pWj2K=
reinterpret_cast<GeoNode*>(RTknE->getChild());if(!pWj2K)return;pWj2K->st2xyh(mS,
mT,mInX,mInY,mInH);mInH+=mOffsetHdg;if(mDir==ODR_DIRECTION_MINUS)mInH+=M_PI;mInZ
=0.0;Elevation*j6B6g=reinterpret_cast<Elevation*>(M8Gux->getFirstElevation());
while(j6B6g&&j6B6g->mSEnd<mS)j6B6g=reinterpret_cast<Elevation*>(j6B6g->getRight(
));if(j6B6g)mInZ=j6B6g->s2z(mS);mInZ+=mZ;UserData*userData=reinterpret_cast<
UserData*>(getChild(ODR_OPCODE_USER_DATA));while(userData){if(userData->mCode==
"\x33"){mReadability=mValue;}else if(userData->mCode=="\x34"){mOcclusion=mValue;
}userData=reinterpret_cast<UserData*>(userData->getRight());}LaneValidity*aVXDm=
reinterpret_cast<LaneValidity*>(getChild(ODR_OPCODE_LANE_VALIDITY));if(aVXDm){
mMinLane=aVXDm->mMinLane;mMaxLane=aVXDm->mMaxLane;}if(mLinkedObjectId.length()){
if(!RoadData::getInstance()){fprintf(stderr,
"\x53\x69\x67\x6e\x61\x6c\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21" "\n" "\x20"
);return;}mLinkedObject=reinterpret_cast<Object*>(RoadData::getInstance()->
getNodeFromId(ODR_OPCODE_OBJECT,mLinkedObjectId));}}Node*Signal::getCopy(bool 
Mupxf){Node*dzamm=new Signal(this);if(Mupxf)deepCopy(dzamm);return dzamm;}void 
Signal::setController(Node*xpKIZ){mController=reinterpret_cast<Controller*>(
xpKIZ);}void Signal::setControlEntry(Node*xpKIZ){mControlEntry=reinterpret_cast<
ControlEntry*>(xpKIZ);}}
