
#include <stdio.h>
#include <math.h>
#include "OdrPath.hh"
#include "OdrPosition.hh"
#include "OdrJuncHeader.hh"
#include "OdrJuncLink.hh"
#include "OdrJuncLaneLink.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include <iostream>
namespace OpenDrive{Path::Leg::LaneList::LaneList():mSRel(0.0){mLaneVec.clear();
}Path::Leg::LaneList::~LaneList(){for(LaneVectorIterator VfDVC=mLaneVec.begin();
VfDVC!=mLaneVec.end();VfDVC++)delete(*VfDVC);mLaneVec.clear();}void Path::Leg::
LaneList::addLane(Lane*lane,bool sLORZ){if(!lane||!lane->mId)return;struct 
LaneInfo*qzPLJ=new struct LaneInfo;qzPLJ->mLane=lane;qzPLJ->mSuccessorLane=0;
qzPLJ->mHasSuccessor=sLORZ;qzPLJ->mRemainingLength=0.0;qzPLJ->mParent=this;qzPLJ
->mSuccLaneInfo=0;qzPLJ->mPredLaneInfo=0;qzPLJ->mIndex=mLaneVec.size();mLaneVec.
push_back(qzPLJ);}void Path::Leg::LaneList::setSRel(const double&BJBDA){mSRel=
BJBDA;}const double&Path::Leg::LaneList::getSRel()const{return mSRel;}void Path
::Leg::LaneList::print(){fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x50\x61\x74\x68\x3a\x3a\x4c\x65\x67\x3a\x3a\x4c\x61\x6e\x65\x4c\x69\x73\x74\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e\x20\x61\x74\x20\x73\x52\x65\x6c\x20\x3d\x20\x25\x2e\x33\x6c\x66\x3a" "\n"
,getSRel());for(LaneVectorIterator VfDVC=mLaneVec.begin();VfDVC!=mLaneVec.end();
VfDVC++)fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x50\x61\x74\x68\x3a\x3a\x4c\x65\x67\x3a\x3a\x4c\x61\x6e\x65\x4c\x69\x73\x74\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x6c\x61\x6e\x65\x20\x25\x2b\x32\x64\x2c\x20\x70\x74\x72\x20\x3d\x20\x25\x70\x2c\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x20\x3d\x20\x25\x70\x2c\x20\x68\x61\x73\x53\x75\x63\x63\x65\x73\x73\x6f\x72\x20\x3d\x20\x25\x64\x2c\x20\x72\x65\x6d\x61\x69\x6e\x69\x6e\x67\x20\x6c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,(*VfDVC)->mLane->mId,(*VfDVC)->mLane,(*VfDVC)->mSuccessorLane,(*VfDVC)->
mHasSuccessor,(*VfDVC)->mRemainingLength);}unsigned int Path::Leg::LaneList::
size(){return mLaneVec.size();}void Path::Leg::LaneList::clear(){return mLaneVec
.clear();}Path::Leg::LaneList::LaneInfo*Path::Leg::LaneList::getInfoAtIndex(
unsigned int hFNbl){if(hFNbl<mLaneVec.size())return mLaneVec.at(hFNbl);return 0;
}Path::Leg::LaneList::LaneInfo*Path::Leg::LaneList::getInfoForLaneId(int laneId)
{for(LaneVectorIterator VfDVC=mLaneVec.begin();VfDVC!=mLaneVec.end();VfDVC++)if(
(*VfDVC)->mLane->mId==laneId)return(*VfDVC);return 0;}Path::Leg::LaneList::
LaneInfo*Path::Leg::LaneList::getNeighborInfo(LaneInfo*MA_pl,int xCtky){if(!
MA_pl)return 0;if(!xCtky)return MA_pl;LaneList*D7gpt=MA_pl->mParent;if(!D7gpt)
return 0;int cGMZP=MA_pl->mIndex+xCtky;if(cGMZP<0)return 0;return getInfoAtIndex
(cGMZP);}void Path::Leg::LaneList::setSuccessorAtIndex(Lane*XPRvc,unsigned int 
hFNbl){if(hFNbl>=mLaneVec.size())return;mLaneVec.at(hFNbl)->mHasSuccessor=true;
mLaneVec.at(hFNbl)->mSuccessorLane=XPRvc;}void Path::Leg::LaneList::
setRemainingLengthAtIndex(const double&len,unsigned int hFNbl){if(hFNbl>=
mLaneVec.size())return;mLaneVec.at(hFNbl)->mRemainingLength=len;}Path::Leg::Leg(
RoadHeader*wR6_y,bool JN4Nc,const double&BJBDA,const double&len):mRoadHdr(wR6_y)
,mForward(JN4Nc),mRoadS(BJBDA),mPathS(0.0),mLen(len){if(!wR6_y)std::cerr<<
"\x20\x20\x20\x20\x50\x61\x74\x68\x3a\x3a\x4c\x65\x67\x3a\x3a\x4c\x65\x67\x20\x3c\x46\x41\x54\x41\x4c\x3a\x20\x4e\x55\x4c\x4c\x20\x72\x6f\x61\x64\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x63\x72\x65\x61\x74\x65\x20\x6c\x65\x67\x3e"
<<std::endl;}Path::Leg::Leg(bool){}Path::Leg::~Leg(){clearLaneLists();}void Path
::Leg::revert(){mForward=!mForward;mRoadS=mLen-mRoadS;}void Path::Leg::print(){
fprintf(stderr,
"\x20\x20\x20\x20\x4c\x65\x67\x3a\x20\x70\x61\x74\x68\x53\x20\x3d\x20\x25\x38\x2e\x33\x6c\x66\x2c\x20\x54\x72\x61\x63\x6b\x20\x3d\x20\x25\x34\x64\x2c\x20\x73\x20\x3d\x20\x25\x38\x2e\x33\x6c\x66\x2c\x20\x6c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x38\x2e\x33\x6c\x66\x2c\x20\x64\x69\x72\x20\x3d\x20\x25\x64" "\n"
,mPathS,mRoadHdr->mId,mRoadS,mLen,mForward?1:-1);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6c\x61\x6e\x65\x4c\x69\x73\x74\x3a" "\n")
;for(LaneListIterator VfDVC=mLaneListVec.begin();VfDVC!=mLaneListVec.end();VfDVC
++)(*VfDVC)->print();}void Path::Leg::calcLaneSequence(const double&byoWw,bool 
kOKsv){if(!mRoadHdr)return;double zkEao=byoWw;LaneSection*H4prs=0;if(mForward){
H4prs=mRoadHdr->getFirstLaneSection();LaneList*jwCSH=0;while(H4prs){LaneSection*
vAn4N=reinterpret_cast<LaneSection*>(H4prs->getRight());if(H4prs->mS>=mRoadS||(
vAn4N&&(vAn4N->mS>mRoadS))||!vAn4N){double ux1Mc=H4prs->mS-mRoadS+byoWw;if(!
kOKsv&&(mRoadS>H4prs->mS))ux1Mc=byoWw+mRoadS;if(ux1Mc<0.0)ux1Mc=0.0;if((ux1Mc-
byoWw)<=mLen){Lane*lane=reinterpret_cast<Lane*>(H4prs->getChild());while(lane&&
lane->getRight())lane=reinterpret_cast<Lane*>(lane->getRight());while(lane){if(!
jwCSH)jwCSH=new LaneList;jwCSH->setSRel(ux1Mc);jwCSH->addLane(lane,false);lane=
reinterpret_cast<Lane*>(lane->getLeft());}}}if(jwCSH){if(jwCSH->size())
mLaneListVec.push_back(jwCSH);else delete jwCSH;}jwCSH=0;H4prs=vAn4N;}}else{
H4prs=mRoadHdr->getLastLaneSection();LaneList*jwCSH=0;zkEao=byoWw;while(H4prs){
LaneSection*vAn4N=reinterpret_cast<LaneSection*>(H4prs->getLeft());if(H4prs->mS
<=mRoadS||!vAn4N){Lane*lane=reinterpret_cast<Lane*>(H4prs->getChild());if(zkEao
>=byoWw&&(zkEao-byoWw)<=mLen){while(lane){if(!jwCSH)jwCSH=new LaneList;jwCSH->
setSRel(zkEao);jwCSH->addLane(lane,false);lane=reinterpret_cast<Lane*>(lane->
getRight());}}}if(jwCSH){if(jwCSH->size())mLaneListVec.push_back(jwCSH);else 
delete jwCSH;}if(jwCSH)zkEao=byoWw+mRoadS-H4prs->mS;jwCSH=0;H4prs=vAn4N;}}}void 
Path::Leg::clearLaneLists(){for(LaneListIterator VfDVC=mLaneListVec.begin();
VfDVC!=mLaneListVec.end();VfDVC++)delete(*VfDVC);mLaneListVec.clear();}unsigned 
int Path::Leg::getSizeLaneList(){return mLaneListVec.size();}Path::Leg::LaneList
*Path::Leg::getLaneListAtIndex(unsigned int hFNbl){return mLaneListVec.at(hFNbl)
;}Path::Leg::LaneList*Path::Leg::s2laneList(const double&BJBDA){for(
LaneListIterator VfDVC=mLaneListVec.begin();VfDVC!=mLaneListVec.end();VfDVC++){
LaneListIterator EW0ka=VfDVC;++EW0ka;if((*VfDVC)->getSRel()<=BJBDA&&(EW0ka==
mLaneListVec.end()||(*EW0ka)->getSRel()>BJBDA))return(*VfDVC);}return 0;}Path::
Path(const std::string&name):mName(name),mCurrentLeg(0),mOnPathOld(false),mSOld(
0.0),mLegOld(0),mWarnPathPos(false){mTotalLen=0.0;mCurrentS=0.0;mStartS=0.0;
mOnPath=true;mRoadQuery=new RoadQuery("\x52\x6f\x61\x64\x51\x75\x65\x72\x79");}
Path::~Path(){clearLegVec(mLegVec);delete mRoadQuery;}const std::string&Path::
getName()const{return mName;}Path*Path::getCopy(){Path*oSyrL=new Path(getName())
;reset();for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=mLegVec.end();ibNLr
++){Leg*xFTsF=new Leg((*ibNLr)->mRoadHdr,(*ibNLr)->mForward,(*ibNLr)->mRoadS,(*
ibNLr)->mLen);oSyrL->addLeg(xFTsF);}oSyrL->calcLaneLists();reset();return oSyrL;
}void Path::reset(){mLanePos=OpenDrive::LaneCoord(0,0,0.0);loop();addS(mStartS);
}void Path::loop(){mCurrentLeg=0;mCurrentS=0.0;mLegIt=mLegVec.begin();mValid=!
mLegVec.empty();if(mValid)mCurrentLeg=*mLegIt;addS(0.0);}bool Path::addWaypoint(
const TrackCoord&KauWq,bool XfG8J){if(mLegVec.empty()){bool JN4Nc=true;if(XfG8J)
{JN4Nc=cos(KauWq.getH())>0.0;}if(addLeg(KauWq.getTrackId(),JN4Nc,KauWq.getS())){
calcLaneLists();return true;}return false;}if(mLegVec.size()==1){reset();
TrackCoord dXv7V(getTrack(),getLegStartS(),0.0);LegVectorIterator VfDVC=mLegVec.
begin();bool JN4Nc=(*VfDVC)->mForward;clearLegVec(mLegVec);mTotalLen=0.0;if(
calcFromTrackPositions(dXv7V,KauWq,XfG8J,JN4Nc)){calcLaneLists();return true;}
return false;}double LR4xi=getLength();TrackCoord dXv7V=getTrackPosAtS(LR4xi);
bool JN4Nc=getFwdAtS(LR4xi);Path yPxNr("\x74\x6d\x70");if(!yPxNr.
calcFromTrackPositions(dXv7V,KauWq,true,JN4Nc)){return false;}yPxNr.reset();if(
getFwdAtS(LR4xi)!=yPxNr.getFwd()){return false;}addPath(yPxNr);calcLaneLists();
return true;}int Path::getTrack()const{if(!mValid)return-1;if(!mCurrentLeg->
mRoadHdr)return-1;return mCurrentLeg->mRoadHdr->mId;}const TrackCoord&Path::
getTrackPos()const{return mLanePos;}const LaneCoord&Path::getLanePos()const{
return mLanePos;}double Path::getLegStartS()const{if(!mValid)return-1.0;return 
mCurrentLeg->mRoadS;}double Path::getS()const{if(!mValid)return-1.0;return 
mCurrentS;}bool Path::getFwd()const{if(!mValid)return false;if(!mCurrentLeg)
return true;return mCurrentLeg->mForward;}TrackCoord Path::getTrackPosAtS(const 
double&BJBDA){TrackCoord HQGFE;double len=0.0;if(mLegVec.empty())return HQGFE;if
(BJBDA<0.0||BJBDA>(mTotalLen+RoadQuery::sMaxDeltaS))return HQGFE;bool dXv7V=fabs
(BJBDA-mTotalLen)<=RoadQuery::sMaxDeltaS;if(dXv7V){LegVectorIterator ibNLr=
mLegVec.end();ibNLr--;if((*ibNLr)->mForward)HQGFE.setS((*ibNLr)->mRoadS+(*ibNLr)
->mLen);else HQGFE.setS((*ibNLr)->mRoadS-(*ibNLr)->mLen);HQGFE.setTrackId((*
ibNLr)->mRoadHdr->mId);return HQGFE;}for(LegVectorIterator ibNLr=mLegVec.begin()
;ibNLr!=mLegVec.end();ibNLr++)if(((*ibNLr)->mLen>0.0)&&((len+(*ibNLr)->mLen)>=
BJBDA)){if((*ibNLr)->mForward)HQGFE.setS(BJBDA-len+(*ibNLr)->mRoadS);else HQGFE.
setS((*ibNLr)->mRoadS-(BJBDA-len));HQGFE.setTrackId((*ibNLr)->mRoadHdr->mId);
return HQGFE;}else len+=(*ibNLr)->mLen;LegVector::reverse_iterator ZrjRs=mLegVec
.rbegin();HQGFE.setTrackId((*ZrjRs)->mRoadHdr->mId);HQGFE.setS((*ZrjRs)->mRoadS)
;return HQGFE;}bool Path::getFwdAtS(const double&BJBDA){Leg*Et_W1=s2Leg(BJBDA);
if(!Et_W1)return true;return Et_W1->mForward;}bool Path::addS(const double&rganP
){bool retVal=false;mOnPath=true;mCurrentS+=rganP;if(mCurrentS<0.0)mCurrentS=0.0
;if(mCurrentS>mTotalLen){mCurrentS=mTotalLen;mOnPath=false;}if(!mOnPath)return 
false;double len=0.0;for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=mLegVec.
end();ibNLr++)if((len+(*ibNLr)->mLen)>=mCurrentS){retVal=(*ibNLr)->mForward!=
mCurrentLeg->mForward;mCurrentLeg=*ibNLr;LaneCoord wxjcS;wxjcS.setTrackId(
mCurrentLeg->mRoadHdr->mId);double rganP=mCurrentS-len;if(!(mCurrentLeg->
mForward))wxjcS.setS(mCurrentLeg->mLen-rganP);else wxjcS.setS(rganP);mLanePos=
wxjcS;return retVal;}else len+=(*ibNLr)->mLen;mOnPath=false;return retVal;}bool 
Path::addOffsetPos(const double&rganP,const int&xCtky,const double&qfVP6){if(
fabs(mCurrentS)<1.0e-3)mCurrentS=0.0;if(mCurrentS+rganP<0.0||mCurrentS+rganP>
mTotalLen){if(!mWarnPathPos)fprintf(stderr,
"\x20\x50\x61\x74\x68\x3a\x3a\x61\x64\x64\x4f\x66\x66\x73\x65\x74\x50\x6f\x73\x3a\x20\x6e\x65\x77\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x65\x78\x63\x65\x65\x64\x73\x20\x70\x61\x74\x68\x3a\x20\x63\x75\x72\x72\x65\x6e\x74\x53\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x64\x73\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x6d\x54\x6f\x74\x61\x6c\x4c\x65\x6e\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,mCurrentS,rganP,mTotalLen);mWarnPathPos=true;return false;}mWarnPathPos=false;
Leg::LaneList::LaneInfo*qzPLJ=pos2LaneInfo();if(!qzPLJ){fprintf(stderr,
"\x20\x50\x61\x74\x68\x3a\x3a\x61\x64\x64\x4f\x66\x66\x73\x65\x74\x50\x6f\x73\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x63\x6f\x6d\x70\x75\x74\x65\x20\x6c\x61\x6e\x65\x20\x69\x6e\x66\x6f" "\n"
);return false;}if(0&&rganP<0.0){fprintf(stderr,
"\x20\x50\x61\x74\x68\x3a\x3a\x61\x64\x64\x4f\x66\x66\x73\x65\x74\x50\x6f\x73\x3a\x20\x63\x61\x6e\x6e\x6f\x74\x20\x79\x65\x74\x20\x68\x61\x6e\x64\x6c\x65\x20\x6e\x65\x67\x61\x74\x69\x76\x65\x20\x64\x73" "\n"
);return false;}Leg::LaneList*D7gpt=qzPLJ->mParent;if(!D7gpt)return false;Leg::
LaneList::LaneInfo*WZ4vt=D7gpt->getNeighborInfo(qzPLJ,xCtky);double L0Em8=
mCurrentS-D7gpt->getSRel();bool zuNde=false;if(qzPLJ->mRemainingLength-L0Em8<
rganP){if(xCtky){zuNde=true;if(!WZ4vt){fprintf(stderr,
"\x20\x50\x61\x74\x68\x3a\x3a\x61\x64\x64\x4f\x66\x66\x73\x65\x74\x50\x6f\x73\x3a\x20\x69\x6d\x70\x6f\x73\x73\x69\x62\x6c\x65\x20\x74\x6f\x20\x63\x68\x61\x6e\x67\x65\x20\x6c\x61\x6e\x65\x20\x66\x69\x72\x73\x74\x21" "\n"
);return false;}qzPLJ=WZ4vt;if(qzPLJ->mRemainingLength-L0Em8<rganP){return false
;}}else{bool zqVA3=false;Leg::LaneList::LaneInfo*fTU3k=qzPLJ;for(int i=-1;!zqVA3
&&i<2;i+=2){qzPLJ=fTU3k;while(!zqVA3&&(qzPLJ=D7gpt->getNeighborInfo(qzPLJ,i)))
zqVA3=(qzPLJ->mRemainingLength-L0Em8)>=rganP;}Path::Leg*hNv3v=s2Leg(mCurrentS+
rganP);if(hNv3v){Path::Leg*tmLif=mCurrentLeg;double pQ3OD=mCurrentS;mCurrentLeg=
hNv3v;mCurrentS=mCurrentS+rganP;Leg::LaneList::LaneInfo*qzPLJ=pos2LaneInfo();if(
qzPLJ){if(addOffsetPos(0.0,0,0))return true;}mCurrentLeg=tmLif;mCurrentS=pQ3OD;}
if(!zqVA3){return false;}}}double RuVdO=mCurrentS+rganP;if(RuVdO>mTotalLen)RuVdO
=mTotalLen;if(RuVdO<0.0)RuVdO=0.0;if(rganP>=0.0){Leg::LaneList::LaneInfo*K6IJc=
qzPLJ->mSuccLaneInfo;while(K6IJc&&(K6IJc->mParent->getSRel()<RuVdO)){qzPLJ=K6IJc
;K6IJc=qzPLJ->mSuccLaneInfo;}}else{Leg::LaneList::LaneInfo*QMJTX=qzPLJ->
mPredLaneInfo;while(QMJTX&&(qzPLJ->mParent->getSRel()>RuVdO)){qzPLJ=QMJTX;QMJTX=
qzPLJ->mPredLaneInfo;}}if(!zuNde)qzPLJ=D7gpt->getNeighborInfo(qzPLJ,xCtky);if(!
qzPLJ){return false;}Lane*lane=qzPLJ->mLane;if(((qzPLJ->mParent->getSRel()+qzPLJ
->mRemainingLength)<RuVdO)){return false;}TrackCoord HQGFE=getTrackPosAtS(RuVdO)
;RoadHeader*M8Gux=reinterpret_cast<RoadHeader*>(lane->getParent()->getParent());
double Z1HuF=getFwdAtS(RuVdO)?1.0:-1.0;double BsLCi=(getFwdAtS(mCurrentS)==
getFwdAtS(RuVdO))?1.0:-1.0;LaneCoord Hv9fs(M8Gux->mId,lane->mId,HQGFE.getS(),
BsLCi*mLanePos.getOffset()+Z1HuF*qfVP6);mLanePos=Hv9fs;mCurrentS=RuVdO;
mCurrentLeg=s2Leg(RuVdO);return true;}void Path::setStartS(const double&BJBDA){
mStartS=BJBDA;}bool Path::setPos(const double&BJBDA){reset();return addOffsetPos
(BJBDA,0,0.0);}double Path::getMaxCurvatureInRange(const double&start,const 
double&_aUXU){double d4xeY=0.0;double _3A7r=mCurrentS+start;double DJ9Y2=
mCurrentS+_aUXU;OpenDrive::Position RyKK9;TrackCoord HQGFE;if(mLegVec.empty())
return 0.0;if((_3A7r<0.0)||(_3A7r>(mTotalLen+RoadQuery::sMaxDeltaS)))return 0.0;
double len=0.0;for(LegVectorIterator ibNLr=mLegVec.begin();(ibNLr!=mLegVec.end()
)&&(len<DJ9Y2);ibNLr++){if(((*ibNLr)->mLen>0.0)&&((len+(*ibNLr)->mLen)>=_3A7r)){
double rODQF=_3A7r-len;double zFWR3=DJ9Y2-len;if((*ibNLr)->mForward){rODQF+=(*
ibNLr)->mRoadS;zFWR3+=(*ibNLr)->mRoadS;}else{rODQF=(*ibNLr)->mRoadS-rODQF;zFWR3=
(*ibNLr)->mRoadS-zFWR3;}double L9KFn=(*ibNLr)->mRoadHdr->getMaxCurvatureInRange(
rODQF,zFWR3);if(fabs(L9KFn)>fabs(d4xeY))d4xeY=L9KFn;}len+=(*ibNLr)->mLen;}return
 d4xeY;}void Path::print(){fprintf(stderr,
"\x50\x61\x74\x68\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x50\x61\x74\x68\x20\x3c\x25\x73\x2c\x20\x25\x70\x3e\x2c\x20\x74\x6f\x74\x61\x6c\x20\x6c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,mName.c_str(),this,mTotalLen);for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr
!=mLegVec.end();ibNLr++)(*ibNLr)->print();fprintf(stderr,
"\x50\x61\x74\x68\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x65\x6e\x64\x20\x6f\x66\x20\x50\x61\x74\x68\x20\x3c\x25\x73\x2c\x20\x25\x70\x3e" "\n"
,mName.c_str(),this);}bool Path::posValid()const{return mOnPath;}bool Path::
calcFromTrackPositions(const TrackCoord&q_y7v,const TrackCoord&KauWq,bool XfG8J,
bool JN4Nc){RoadQuery Ujqjw("\x64\x75\x6d\x6d\x79");RoadHeader*REKut=Ujqjw.
trackId2Node(q_y7v.getTrackId());if(!REKut){std::cerr<<
"\x50\x61\x74\x68\x3a\x3a\x63\x61\x6c\x63\x46\x72\x6f\x6d\x54\x72\x61\x63\x6b\x50\x6f\x73\x69\x74\x69\x6f\x6e\x73\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x6c\x6f\x63\x61\x74\x65\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x74\x72\x61\x63\x6b\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x2e\x20\x43\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64"
<<std::endl;std::cerr<<"\x74\x72\x61\x63\x6b\x49\x64\x20\x3d\x20"<<q_y7v.
getTrackId()<<std::endl;return false;}if(q_y7v.getTrackId()==KauWq.getTrackId())
{bool d0YFF=(JN4Nc&&(KauWq.getS()<q_y7v.getS()))||(!JN4Nc&&(KauWq.getS()>q_y7v.
getS()));bool Eo1eX=fabs(KauWq.getS()-q_y7v.getS())<1.0e-2||(fabs(REKut->mLength
-(fabs(KauWq.getS()-q_y7v.getS())))<1.0e-2);if((REKut->getPredecessor()!=REKut)
||(REKut->getSuccessor()!=REKut)){d0YFF=false;Eo1eX=false;}if(Eo1eX){if((REKut->
getPredecessor()!=REKut)||(REKut->getSuccessor()!=REKut))return false;if(JN4Nc)
addLeg(q_y7v.getTrackId(),JN4Nc,REKut->mLength);else addLeg(q_y7v.getTrackId(),
JN4Nc,0.0);addLeg(KauWq.getTrackId(),JN4Nc,KauWq.getS());}else if(d0YFF){if((
REKut->getPredecessor()!=REKut)||(REKut->getSuccessor()!=REKut))return false;if(
mLegVec.empty())addLeg(q_y7v.getTrackId(),JN4Nc,q_y7v.getS());if(JN4Nc){addLeg(
q_y7v.getTrackId(),JN4Nc,REKut->mLength);if(KauWq.getS()!=0.0){addLeg(q_y7v.
getTrackId(),JN4Nc,0.0);addLeg(KauWq.getTrackId(),JN4Nc,KauWq.getS());}}else{
addLeg(q_y7v.getTrackId(),JN4Nc,0.0);if(fabs(KauWq.getS()-REKut->mLength)>1.0e-3
){addLeg(q_y7v.getTrackId(),JN4Nc,REKut->mLength);addLeg(KauWq.getTrackId(),
JN4Nc,KauWq.getS());}}}else{bool forward=XfG8J?JN4Nc:KauWq.getS()>q_y7v.getS();
if((forward&&(KauWq.getS()<q_y7v.getS()))||(!forward&&(KauWq.getS()>q_y7v.getS()
)))return false;addLeg(q_y7v.getTrackId(),forward,q_y7v.getS());addLeg(KauWq.
getTrackId(),forward,KauWq.getS());}return true;}bool forward=XfG8J?JN4Nc:true;
if(checkConnectionsForTargetPos(REKut,KauWq,forward,0,0,0,false)){addLeg(q_y7v.
getTrackId(),forward,q_y7v.getS());invertOrder();if(!XfG8J||(XfG8J&&(forward!=
JN4Nc))){forward=XfG8J?JN4Nc:true;LegVector lWmQh=mLegVec;mLegVec.clear();double
 aiJrf=mTotalLen;mTotalLen=0.0;if(checkConnectionsForTargetPos(REKut,KauWq,
forward,0,0,0,true)){addLeg(q_y7v.getTrackId(),forward,q_y7v.getS());invertOrder
();if(!XfG8J&&(mTotalLen>aiJrf)){clearLegVec(mLegVec);mLegVec=lWmQh;lWmQh.clear(
);mTotalLen=aiJrf;}else clearLegVec(lWmQh);}else{clearLegVec(mLegVec);mLegVec=
lWmQh;lWmQh.clear();mTotalLen=aiJrf;}}return true;}return false;}void Path::
invert(){invertOrder();for(LegVectorIterator VfDVC=mLegVec.begin();VfDVC!=
mLegVec.end();VfDVC++)(*VfDVC)->revert();calcLaneLists();}const double&Path::
getLength()const{return mTotalLen;}bool Path::sameTrackDir(const double&iL5Gj,
const double&ikhot){Leg*KM6rl=s2Leg(iL5Gj);Leg*FuPcU=s2Leg(ikhot);if(!KM6rl||!
FuPcU){return true;}return KM6rl->mForward==FuPcU->mForward;}bool Path::addPath(
Path*japsW){if(!japsW)return false;japsW->reset();do{addLeg(japsW->getTrack(),
japsW->getFwd(),japsW->getLegStartS());}while(japsW->nextLeg());optimize();
return true;}bool Path::addPath(Path&japsW){return addPath(&japsW);}bool Path::
pos2s(const Coord&wiBPl){mInertialPos=wiBPl;mOnPath=false;LaneCoord::LaneVec 
D7gpt;int result=mRoadQuery->inertial2lane(mInertialPos,D7gpt);if(result!=
RoadQuery::RESULT_ON_ROAD)return false;for(OpenDrive::LaneCoord::LaneVec::
iterator VfDVC=D7gpt.begin();VfDVC!=D7gpt.end()&&!mOnPath;VfDVC++)if(pos2s(*
VfDVC))return true;return false;}bool Path::pos2s(const LaneCoord&Hv9fs,const 
double&Yrh_j){mOnPath=false;mCurrentS=0.0;for(int i=0;i<2;i++){mCurrentLeg=
pos2Leg(Hv9fs,Yrh_j,i==1);if(!mCurrentLeg)return false;mCurrentS=mCurrentLeg->
mPathS;if(mCurrentLeg->mForward)mCurrentS+=Hv9fs.getS()-mCurrentLeg->mRoadS;else
 mCurrentS+=mCurrentLeg->mRoadS-Hv9fs.getS();if(mCurrentS>=Yrh_j)break;}mLanePos
=Hv9fs;mOnPath=true;mValid=true;return true;}bool Path::nextLeg(){if(mLegIt!=
mLegVec.end())mLegIt++;if(mLegIt==mLegVec.end()){mValid=false;mCurrentLeg=0;
return false;}mCurrentLeg=*mLegIt;return true;}void Path::getRoads(std::vector<
RoadHeader*>&y97wI,const double&Yrh_j){y97wI.clear();for(LegVectorIterator VfDVC
=mLegVec.begin();VfDVC!=mLegVec.end();VfDVC++){if(((*VfDVC)->mPathS+(*VfDVC)->
mLen)>=Yrh_j)y97wI.push_back((*VfDVC)->mRoadHdr);}}void Path::pushPos(){
mOnPathOld=mOnPath;mLanePosOld=mLanePos;mInertialPosOld=mInertialPos;mLegOld=
mCurrentLeg;mSOld=mCurrentS;}void Path::popPos(){mOnPath=mOnPathOld;mLanePos=
mLanePosOld;mInertialPos=mInertialPosOld;mCurrentLeg=mLegOld;mCurrentS=mSOld;}
void Path::clearLegVec(std::vector<Leg*>&HcddB){for(LegVectorIterator VfDVC=
HcddB.begin();VfDVC!=HcddB.end();++VfDVC){delete(*VfDVC);}HcddB.clear();}void 
Path::cleanCopyLegVec(LegVector&szJ7I,LegVector&SkShA,LegVector&BLJaz){for(
LegVectorIterator VfDVC=szJ7I.begin();VfDVC!=szJ7I.end();++VfDVC){bool DSqba=
false;for(LegVectorIterator EJ1xB=SkShA.begin();EJ1xB!=SkShA.end()&&!DSqba;++
EJ1xB)DSqba=*VfDVC==*EJ1xB;if(!DSqba){for(LegVectorIterator EJ1xB=BLJaz.begin();
EJ1xB!=BLJaz.end()&&!DSqba;++EJ1xB)DSqba=*VfDVC==*EJ1xB;if(!DSqba)delete(*VfDVC)
;}}szJ7I.clear();szJ7I=SkShA;}bool Path::addLeg(Leg*UuV75){if(!UuV75)return 
false;UuV75->mPathS=mTotalLen;mLegVec.push_back(UuV75);mTotalLen+=UuV75->mLen;
mCurrentLeg=UuV75;return true;}Path::Leg*Path::getCurrentLeg()const{return 
mCurrentLeg;}bool Path::addLeg(unsigned int LqMUn,bool JN4Nc,const double&BJBDA)
{return addLeg(mRoadQuery->trackId2Node(LqMUn),JN4Nc,BJBDA);}bool Path::addLeg(
RoadHeader*wR6_y,bool JN4Nc,const double&BJBDA){if(!wR6_y)return false;double 
WLy47=JN4Nc?wR6_y->mLength-BJBDA:BJBDA;LegVector::reverse_iterator VfDVC=mLegVec
.rbegin();if(VfDVC!=mLegVec.rend()){Leg*UuV75=*VfDVC;if(UuV75->mRoadHdr==wR6_y&&
UuV75->mForward==JN4Nc){double ElF7p=fabs(BJBDA-UuV75->mRoadS);bool UANii=(wR6_y
->getPredecessor()==wR6_y)||(wR6_y->getSuccessor()==wR6_y);if(!UANii||((JN4Nc&&(
ElF7p+UuV75->mRoadS)<=UuV75->mRoadHdr->mLength)||(!JN4Nc&&(UuV75->mRoadS-ElF7p)
>=0.0))){mTotalLen-=UuV75->mLen;UuV75->mLen=fabs(BJBDA-UuV75->mRoadS);if((fabs(
UuV75->mLen)<RoadQuery::sMaxDeltaS)){UuV75->mLen=WLy47;mTotalLen+=UuV75->mLen;
return true;}mTotalLen+=UuV75->mLen;WLy47=0.0;}}}else WLy47=0.0;Leg*xFTsF=new 
Leg(wR6_y,JN4Nc,BJBDA,WLy47);if(!xFTsF->mRoadHdr){delete xFTsF;return false;}
xFTsF->mPathS=mTotalLen;mLegVec.push_back(xFTsF);mTotalLen+=WLy47;mTotalLen=
xFTsF->mPathS+WLy47;mCurrentLeg=xFTsF;return true;}bool Path::addLeg(unsigned 
int LqMUn,bool JN4Nc){return addLeg(mRoadQuery->trackId2Node(LqMUn),JN4Nc);}bool
 Path::addLeg(RoadHeader*wR6_y,bool JN4Nc){if(!wR6_y)return false;double BJBDA=
0.0;if(!JN4Nc)BJBDA=wR6_y->mLength;return addLeg(wR6_y,JN4Nc,BJBDA);}bool Path::
addLeg(unsigned int LqMUn){return addLeg(mRoadQuery->trackId2Node(LqMUn));}bool 
Path::addLeg(RoadHeader*wR6_y){if(!wR6_y)return false;if(mLegVec.empty())return 
addLeg(wR6_y,true);Position dXv7V;dXv7V.setPos(getTrackPosAtS(mTotalLen-
RoadQuery::sMaxDeltaS));dXv7V.track2inertial();Position rHsiY;rHsiY.setTrackPos(
wR6_y->mId,0.0);rHsiY.track2inertial();if(Position::getInertialDist(rHsiY,dXv7V)
<0.01)return addLeg(wR6_y,true);if(mLegVec.size()==1){LegVectorIterator VfDVC=
mLegVec.begin();dXv7V.setPos(getTrackPosAtS(0.0));dXv7V.track2inertial();if(
Position::getInertialDist(rHsiY,dXv7V)<0.01){(*VfDVC)->revert();return addLeg(
wR6_y,true);}rHsiY.setTrackPos(wR6_y->mId,wR6_y->mLength);rHsiY.track2inertial()
;if(Position::getInertialDist(rHsiY,dXv7V)<0.01){(*VfDVC)->revert();return 
addLeg(wR6_y,false);}}return addLeg(wR6_y,false);}bool Path::
checkConnectionsForTargetPos(RoadHeader*REKut,const TrackCoord&KauWq,bool&
forward,RoadHeader*II6ml,int depth,int ls_cG,bool ChBDm){if(!REKut)return false;
if(depth>0)ChBDm=false;if(depth>=sMaxSearchDepth){std::cerr<<
"\x50\x61\x74\x68\x3a\x3a\x63\x68\x65\x63\x6b\x43\x6f\x6e\x6e\x65\x63\x74\x69\x6f\x6e\x73\x46\x6f\x72\x54\x61\x72\x67\x65\x74\x50\x6f\x73\x3a\x20\x6d\x61\x78\x2e\x20\x73\x65\x61\x72\x63\x68\x20\x64\x65\x70\x74\x68\x20\x72\x65\x61\x63\x68\x65\x64\x2e\x20\x47\x69\x76\x69\x6e\x67\x20\x75\x70\x2e"
<<std::endl;return false;}if(REKut->mId==(unsigned int)(KauWq.getTrackId())){
bool snKdw=true;if(II6ml){if(II6ml->getPredecessor()==REKut&&II6ml->getSuccessor
()==REKut){bool JN4Nc=mCurrentLeg?mCurrentLeg->mForward:forward;if(JN4Nc){snKdw=
II6ml->getSuccessorDir()==ODR_LINK_POINT_START;forward=true;}else{snKdw=II6ml->
getPredecessorDir()==ODR_LINK_POINT_START;forward=false;}}else{if(II6ml->
getPredecessor()==REKut)snKdw=II6ml->getPredecessorDir()==ODR_LINK_POINT_START;
if(II6ml->getSuccessor()==REKut)snKdw=II6ml->getSuccessorDir()==
ODR_LINK_POINT_START;Node*node=REKut->getSuccessorNode();if(node&&node->
getOpcode()==ODR_OPCODE_ROAD_HEADER){if(REKut->getSuccessor()==II6ml)snKdw=false
;}node=REKut->getPredecessorNode();if(node&&node->getOpcode()==
ODR_OPCODE_ROAD_HEADER){if(REKut->getPredecessor()==II6ml)snKdw=true;}}}addLeg(
KauWq.getTrackId(),snKdw,KauWq.getS());if(snKdw){if(KauWq.getS()!=0.0)addLeg(
KauWq.getTrackId(),snKdw);}else{RoadHeader*M8Gux=mRoadQuery->trackId2Node(KauWq.
getTrackId());if(M8Gux){if(fabs(KauWq.getS()-M8Gux->mLength)>1.0e-6)addLeg(M8Gux
,snKdw);}}return true;}for(int i=0;i<2;i++){Node*node;bool getSuccessor=(i&&!
ChBDm)||(!i&&ChBDm);if(getSuccessor)node=REKut->getSuccessorNode();else node=
REKut->getPredecessorNode();if(node){switch(node->getOpcode()){case 
ODR_OPCODE_ROAD_HEADER:{RoadHeader*NGRHf=reinterpret_cast<RoadHeader*>(node);if(
NGRHf&&NGRHf!=II6ml){if(checkConnectionsForTargetPos(NGRHf,KauWq,forward,REKut,
depth+1,ls_cG)){if(REKut->getPredecessor()!=REKut->getSuccessor())forward=
getSuccessor;if(II6ml)addLeg(REKut->mId,forward);return true;}}}break;case 
ODR_OPCODE_JUNCTION_HEADER:if(!ls_cG){JuncHeader*GqNs7=reinterpret_cast<
JuncHeader*>(node);JuncLink*hYfdO=GqNs7->getFirstLink();double pmUGY=mTotalLen;
LegVector yuyY1=mLegVec;double TcqTF=mTotalLen;LegVector mZhMz=mLegVec;double 
nTHqB=-1.0;bool XfByx=false;bool gRRKl=false;while(hYfdO&&!gRRKl){gRRKl=hYfdO->
mConnectingRoad==II6ml;hYfdO=reinterpret_cast<JuncLink*>(hYfdO->getRight());}if(
gRRKl)break;hYfdO=GqNs7->getFirstLink();while(hYfdO){if(hYfdO->mIncomingRoad==
REKut&&hYfdO->mConnectingRoad!=II6ml){if(checkConnectionsForTargetPos(hYfdO->
mConnectingRoad,KauWq,forward,REKut,depth+1,ls_cG+1)){forward=getSuccessor;if(
II6ml)addLeg(REKut->mId,forward);if(nTHqB<0.0||(mTotalLen<nTHqB)){nTHqB=
mTotalLen;pmUGY=mTotalLen;yuyY1=mLegVec;}cleanCopyLegVec(mLegVec,mZhMz,yuyY1);
mZhMz.clear();mTotalLen=TcqTF;XfByx=true;}}hYfdO=reinterpret_cast<JuncLink*>(
hYfdO->getRight());}if(XfByx){cleanCopyLegVec(mLegVec,yuyY1,yuyY1);yuyY1.clear()
;mTotalLen=pmUGY;return true;}}break;}}}return false;}void Path::invertOrder(){
int size=mLegVec.size();for(int i=0;i<(size/2);i++){Leg*hNv3v=mLegVec[i];mLegVec
[i]=mLegVec[size-i-1];mLegVec[size-i-1]=hNv3v;}double byoWw=0.0;for(int i=0;i<
size-1;i++){if(mLegVec[i+1]->mRoadHdr==mLegVec[i]->mRoadHdr){mLegVec[i]->mLen=
fabs(mLegVec[i+1]->mRoadS-mLegVec[i]->mRoadS);mLegVec[i+1]->mLen=0.0;}mLegVec[i]
->mPathS=byoWw;byoWw+=mLegVec[i]->mLen;}}Path::Leg*Path::s2Leg(const double&
BJBDA){double len=0.0;if(BJBDA>(mTotalLen+RoadQuery::sMaxDeltaS)||BJBDA<0.0)
return 0;if(mLegVec.empty())return 0;if(fabs(BJBDA-mTotalLen)<=RoadQuery::
sMaxDeltaS)return(*(mLegVec.rbegin()));for(mLegIt=mLegVec.begin();mLegIt!=
mLegVec.end();mLegIt++){LegVectorIterator EW0ka=mLegIt;EW0ka++;if(((len+(*mLegIt
)->mLen)>=BJBDA)||EW0ka==mLegVec.end())return(*mLegIt);len+=(*mLegIt)->mLen;}
return(*(mLegVec.rbegin()));}Path::Leg*Path::pos2Leg(const LaneCoord&Hv9fs,const
 double&Yrh_j,bool TE61x){if(TE61x){for(LegVectorIterator ibNLr=mLegVec.begin();
ibNLr!=mLegVec.end();ibNLr++)if((*ibNLr)->mRoadHdr->mId==(unsigned int)(Hv9fs.
getTrackId())&&(((*ibNLr)->mPathS)>=Yrh_j)){if((((*ibNLr)->mForward)&&(Hv9fs.
getS()>=(*ibNLr)->mRoadS))||(!((*ibNLr)->mForward)&&(Hv9fs.getS()<=(*ibNLr)->
mRoadS)))return(*ibNLr);}}if(mCurrentLeg&&(mCurrentLeg->mRoadHdr->mId==(unsigned
 int)(Hv9fs.getTrackId()))&&((mCurrentLeg->mPathS+mCurrentLeg->mLen)>=Yrh_j)&&(
mCurrentLeg->mPathS<=Yrh_j))return mCurrentLeg;for(LegVectorIterator ibNLr=
mLegVec.begin();ibNLr!=mLegVec.end();ibNLr++)if((*ibNLr)->mRoadHdr->mId==(
unsigned int)(Hv9fs.getTrackId())&&(((*ibNLr)->mPathS+(*ibNLr)->mLen)>=Yrh_j))
return(*ibNLr);return 0;}void Path::optimize(){LegVectorIterator gjSdP=mLegVec.
end();LegVectorIterator pIZ4R;for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr
!=mLegVec.end();ibNLr++){pIZ4R=ibNLr;pIZ4R++;if((ibNLr!=mLegVec.begin())&&(pIZ4R
!=mLegVec.end())){if((*gjSdP)->mRoadHdr==(*ibNLr)->mRoadHdr&&(*gjSdP)->mForward
==(*ibNLr)->mForward){if(((*ibNLr)->mForward&&((*gjSdP)->mRoadS<(*ibNLr)->mRoadS
))||(!((*ibNLr)->mForward)&&((*gjSdP)->mRoadS>(*ibNLr)->mRoadS))){(*gjSdP)->mLen
+=(*ibNLr)->mLen;delete*ibNLr;ibNLr=mLegVec.erase(ibNLr);ibNLr--;}}}gjSdP=ibNLr;
}}void Path::calcLaneLists(){for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=
mLegVec.end();ibNLr++)(*ibNLr)->clearLaneLists();double byoWw=0.0;bool kOKsv=
true;for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=mLegVec.end();ibNLr++){
if(((*ibNLr)->mLen>0.0)||((ibNLr==mLegVec.begin())&&((*ibNLr)->mLen==0.0))){(*
ibNLr)->calcLaneSequence(byoWw,kOKsv);kOKsv=false;}(*ibNLr)->mPathS=byoWw;byoWw
+=(*ibNLr)->mLen;}updateLaneValidity();calcLaneLengths();Leg::LaneList*WsmL9=0;
Leg::LaneList*D7gpt=0;for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=mLegVec
.end();ibNLr++){for(Leg::LaneListIterator VfDVC=(*ibNLr)->mLaneListVec.begin();
VfDVC!=(*ibNLr)->mLaneListVec.end();VfDVC++){D7gpt=(*VfDVC);if(WsmL9){for(
unsigned int FLneO=0;FLneO<WsmL9->size();++FLneO){Leg::LaneList::LaneInfo*tcuL7=
WsmL9->getInfoAtIndex(FLneO);tcuL7->mSuccLaneInfo=0;for(unsigned int cGMZP=0;
cGMZP<D7gpt->size();++cGMZP){Leg::LaneList::LaneInfo*qzPLJ=D7gpt->getInfoAtIndex
(cGMZP);if(qzPLJ->mLane==tcuL7->mSuccessorLane){tcuL7->mSuccLaneInfo=qzPLJ;qzPLJ
->mPredLaneInfo=tcuL7;}}}}WsmL9=D7gpt;}}reset();}void Path::updateLaneValidity()
{for(LegVectorIterator ibNLr=mLegVec.begin();ibNLr!=mLegVec.end();ibNLr++){
LegVectorIterator pIZ4R=ibNLr;pIZ4R++;for(unsigned int i=0;i<(*ibNLr)->
getSizeLaneList();i++){Leg::LaneList*XF3fW=(*ibNLr)->getLaneListAtIndex(i);Leg::
LaneList*ACm5h=0;bool qqGdl=false;if(i<(*ibNLr)->getSizeLaneList()-1)ACm5h=(*
ibNLr)->getLaneListAtIndex(i+1);else if(pIZ4R!=mLegVec.end()){if((*pIZ4R)->
getSizeLaneList()){ACm5h=(*pIZ4R)->getLaneListAtIndex(0);qqGdl=true;}else return
;}else return;if(qqGdl){RoadHeader*wR6_y=(*pIZ4R)->mRoadHdr;if(wR6_y&&wR6_y->
getJunction()){JuncHeader*GqNs7=reinterpret_cast<JuncHeader*>(wR6_y->getJunction
());JuncLink*hYfdO=GqNs7->getFirstLink();while(hYfdO){if((hYfdO->mIncomingRoad==
(*ibNLr)->mRoadHdr)&&(hYfdO->mConnectingRoad==wR6_y)){JuncLaneLink*SJBOF=hYfdO->
getFirstLaneLink();while(SJBOF){for(unsigned int lYwOM=0;lYwOM<XF3fW->size();++
lYwOM){Lane*Le97U=XF3fW->getInfoAtIndex(lYwOM)->mLane;if(Le97U==SJBOF->
mIncomingLane)XF3fW->setSuccessorAtIndex(SJBOF->mConnectingLane,lYwOM);}SJBOF=
reinterpret_cast<JuncLaneLink*>(SJBOF->getRight());}}hYfdO=reinterpret_cast<
JuncLink*>(hYfdO->getRight());}}}for(unsigned int lYwOM=0;lYwOM<XF3fW->size();++
lYwOM){Lane*Le97U=XF3fW->getInfoAtIndex(lYwOM)->mLane;for(unsigned int j=0;j<
ACm5h->size();++j){Lane*M8HSW=ACm5h->getInfoAtIndex(j)->mLane;if(Le97U->
getPredecessor()==M8HSW||Le97U->getSuccessor()==M8HSW)XF3fW->setSuccessorAtIndex
(M8HSW,lYwOM);}}}}}void Path::calcLaneLengths(){Leg::LaneList*WsmL9=0;for(
LegVector::reverse_iterator ibNLr=mLegVec.rbegin();ibNLr!=mLegVec.rend();ibNLr++
){for(Leg::LaneListVector::reverse_iterator RLNb0=(*ibNLr)->mLaneListVec.rbegin(
);RLNb0!=(*ibNLr)->mLaneListVec.rend();RLNb0++){Leg::LaneList*D7gpt=(*RLNb0);for
(Leg::LaneList::LaneVectorIterator SWDB_=D7gpt->mLaneVec.begin();SWDB_!=D7gpt->
mLaneVec.end();SWDB_++){Leg::LaneList::LaneInfo*qzPLJ=*SWDB_;if(qzPLJ->
mSuccessorLane){if(WsmL9){for(Leg::LaneList::LaneVectorIterator KIHB9=WsmL9->
mLaneVec.begin();KIHB9!=WsmL9->mLaneVec.end();KIHB9++){Leg::LaneList::LaneInfo*
tcuL7=*KIHB9;if(qzPLJ->mSuccessorLane==tcuL7->mLane){qzPLJ->mRemainingLength=
tcuL7->mRemainingLength+(WsmL9->getSRel()-D7gpt->getSRel());}}}}else if(WsmL9)
qzPLJ->mRemainingLength=WsmL9->getSRel()-D7gpt->getSRel();else qzPLJ->
mRemainingLength=mTotalLen-D7gpt->getSRel();}WsmL9=D7gpt;}}}Path::Leg::LaneList
::LaneInfo*Path::pos2LaneInfo(){if(!mCurrentLeg){std::cerr<<
"\x50\x61\x74\x68\x3a\x3a\x70\x6f\x73\x32\x4c\x61\x6e\x65\x49\x6e\x66\x6f\x3a\x20\x63\x61\x6e\x6e\x6f\x74\x20\x77\x6f\x72\x6b\x20\x6f\x6e\x20\x75\x6e\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x20\x70\x61\x74\x68\x20"
<<std::endl;return 0;}Leg::LaneList*D7gpt=mCurrentLeg->s2laneList(mCurrentS);if(
!D7gpt)mLanePos.print();if(!D7gpt)return 0;Path::Leg::LaneList::LaneInfo*GrPwf=
D7gpt->getInfoForLaneId(mLanePos.getLaneId());if(GrPwf)return GrPwf;GrPwf=D7gpt
->getInfoForLaneId(-1);if(GrPwf)return GrPwf;return D7gpt->getInfoAtIndex(0);}}
