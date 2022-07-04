
#include "OdrNode.hh"
#include "OdrReaderXML.hh"
#include "OdrParserCallback.hh"
#include <iostream>
#include <stdio.h>
#include <math.h>
namespace OpenDrive{Node::Node(const std::string&name):mIdentifier(name),mLeft(0
),mRight(0),mParent(0),mData(0),mCDATA(""),mDebugInfo(0),mLineNo(0),mOpcode(0),
mLevel(0){}Node::Node(Node*Nf2Ao){if(!Nf2Ao)return;mIdentifier=Nf2Ao->getName();
mLeft=0;mRight=0;mParent=0;mData=Nf2Ao->getData();mDebugInfo=Nf2Ao->getDebugInfo
();mLineNo=Nf2Ao->getLineNo();mOpcode=Nf2Ao->getOpcode();mLevel=Nf2Ao->getLevel(
);mCDATA=Nf2Ao->getCDATA();mDebugInfo|=sFlagClone;}Node::~Node(){while(!
mChildMap.empty()){NodeMap::iterator VfDVC=mChildMap.begin();delete VfDVC->
second;}if(mLeft)mLeft->setRight(mRight);if(mRight)mRight->setLeft(mLeft);if(!
mParent)return;if(mParent->getChild(mOpcode)==this)mParent->setChild(mRight,
mOpcode);}const std::string&Node::getName()const{return mIdentifier;}unsigned 
int Node::getOpcode()const{return mOpcode;}unsigned int Node::getLevel()const{
return mLevel;}Node*Node::getLeft(){return mLeft;}const Node*Node::getLeft()
const{return mLeft;}Node*Node::getRight(){return mRight;}const Node*Node::
getRight()const{return mRight;}Node*Node::getParent(){return mParent;}const Node
*Node::getParent()const{return mParent;}unsigned int Node::getNumChildTypes()
const{return mChildMap.size();}Node*Node::getChild(unsigned int CvVIp){NodeMap::
iterator VfDVC=mChildMap.find(CvVIp);if(VfDVC==mChildMap.end())return 0;return 
VfDVC->second;}Node*Node::getLastChild(unsigned int CvVIp){NodeMap::iterator 
VfDVC=mLastChildMap.find(CvVIp);if(VfDVC==mLastChildMap.end())return 0;return 
VfDVC->second;}Node*Node::getChild(){NodeMap::iterator VfDVC=mChildMap.begin();
if(VfDVC==mChildMap.end())return 0;return VfDVC->second;}Node*Node::
getChildAtTypeIdx(unsigned int hFNbl){if(hFNbl>=mChildMap.size())return 0;
NodeMap::iterator VfDVC=mChildMap.begin();for(unsigned int i=0;i<hFNbl;++i)VfDVC
++;return VfDVC->second;}void Node::addChild(Node*rkAEh,bool first){NodeMap::
iterator VfDVC;VfDVC=mChildMap.find(rkAEh->getOpcode());if(VfDVC==mChildMap.end(
)){std::pair<NodeMap::iterator,bool>QVavK=mChildMap.insert(std::pair<unsigned 
int,Node*>(rkAEh->getOpcode(),rkAEh));if(!QVavK.second)std::cerr<<
"\x4e\x6f\x64\x65\x3a\x3a\x61\x64\x64\x43\x68\x69\x6c\x64\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x61\x64\x64\x20\x63\x68\x69\x6c\x64\x20\x6e\x6f\x64\x65\x20\x74\x6f\x20\x6d\x61\x70\x21"
<<std::endl;rkAEh->setParent(this);QVavK=mLastChildMap.insert(std::pair<unsigned
 int,Node*>(rkAEh->getOpcode(),rkAEh));if(!QVavK.second)std::cerr<<
"\x4e\x6f\x64\x65\x3a\x3a\x61\x64\x64\x43\x68\x69\x6c\x64\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x61\x64\x64\x20\x6c\x61\x73\x74\x20\x63\x68\x69\x6c\x64\x20\x6e\x6f\x64\x65\x20\x74\x6f\x20\x6d\x61\x70\x21"
<<std::endl;return;}if(first){VfDVC->second->addLeftSibling(rkAEh);return;}
NodeMap::iterator veUZd=mLastChildMap.find(rkAEh->getOpcode());if(veUZd==
mLastChildMap.end()){Node*Gv01F=VfDVC->second;while(Gv01F->getRight())Gv01F=
Gv01F->getRight();Gv01F->addRightSibling(rkAEh);}else{Node*Gv01F=veUZd->second;
Gv01F->addRightSibling(rkAEh);}}void Node::addLeftSibling(Node*Gv01F){Node*gb79w
=mLeft;mLeft=Gv01F;Gv01F->setRight(this);Gv01F->setParent(mParent);if(!gb79w){if
(mParent)mParent->setChild(Gv01F,Gv01F->getOpcode());return;}gb79w->setRight(
Gv01F);Gv01F->setLeft(gb79w);}void Node::addRightSibling(Node*Gv01F){Node*xvWkN=
mRight;mRight=Gv01F;Gv01F->setLeft(this);Gv01F->setParent(mParent);if(mParent&&!
xvWkN){NodeMap::iterator veUZd=mParent->mLastChildMap.find(Gv01F->getOpcode());
if(veUZd==mParent->mLastChildMap.end()){fprintf(stderr,
"\x4e\x6f\x64\x65\x3a\x3a\x61\x64\x64\x52\x69\x67\x68\x74\x53\x69\x62\x6c\x69\x6e\x67\x3a\x20\x57\x41\x52\x4e\x49\x4e\x47\x3a\x20\x70\x61\x72\x65\x6e\x74\x20\x6e\x6f\x64\x65\x20\x64\x6f\x65\x73\x20\x6e\x6f\x74\x20\x68\x61\x76\x65\x20\x72\x65\x67\x69\x73\x74\x72\x61\x74\x69\x6f\x6e\x20\x66\x6f\x72\x20\x6c\x61\x73\x74\x20\x73\x69\x62\x6c\x69\x6e\x67\x21\x20\x54\x68\x69\x73\x20\x73\x68\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x68\x61\x70\x70\x65\x6e" "\n"
);std::pair<NodeMap::iterator,bool>QVavK=mParent->mLastChildMap.insert(std::pair
<unsigned int,Node*>(Gv01F->getOpcode(),Gv01F));if(!QVavK.second)std::cerr<<
"\x4e\x6f\x64\x65\x3a\x3a\x61\x64\x64\x52\x69\x67\x68\x74\x53\x69\x62\x6c\x69\x6e\x67\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x61\x64\x64\x20\x6c\x61\x73\x74\x20\x63\x68\x69\x6c\x64\x20\x6e\x6f\x64\x65\x20\x74\x6f\x20\x6d\x61\x70\x21"
<<std::endl;}else veUZd->second=Gv01F;return;}xvWkN->setLeft(Gv01F);Gv01F->
setRight(xvWkN);}void Node::setLeft(Node*Gv01F){mLeft=Gv01F;}void Node::setRight
(Node*Gv01F){mRight=Gv01F;}void Node::setParent(Node*parent){mParent=parent;}
bool Node::shiftRight(){if(!mRight)return false;if(mLeft)mLeft->setRight(mRight)
;else mParent->setChild(mRight,mOpcode);mRight->setLeft(mLeft);mLeft=mRight;
mRight=mRight->getRight();if(mRight)mRight->setLeft(this);mLeft->setRight(this);
return true;}void Node::print(bool Mupxf,bool KvFxK){for(unsigned int i=0;i<
mLevel;++i)fprintf(stderr,"\x20\x20\x20\x20");
#if 1
fprintf(stderr,
"\x44\x61\x74\x61\x20\x6f\x66\x20\x3c\x25\x73\x3e\x25\x73\x2c\x20\x66\x72\x6f\x6d\x20\x73\x6f\x75\x72\x63\x65\x20\x6c\x69\x6e\x65\x20\x25\x64\x2c\x20\x66\x69\x6c\x65\x20\x6e\x6f\x2e\x20\x25\x64\x3a" "\n"
,mIdentifier.c_str(),isCloned()?"\x28\x63\x6c\x6f\x6e\x65\x29":"",getLineNo(),
getSequenceNo());
#else
fprintf(stderr,
"\x44\x61\x74\x61\x20\x6f\x66\x20\x3c\x25\x73\x3e\x25\x73\x2c\x20\x66\x72\x6f\x6d\x20\x73\x6f\x75\x72\x63\x65\x20\x6c\x69\x6e\x65\x20\x25\x64\x2c\x20\x66\x69\x6c\x65\x20\x6e\x6f\x2e\x20\x25\x64\x2c\x20\x74\x68\x69\x73\x3d\x25\x70\x2c\x20\x70\x61\x72\x65\x6e\x74\x3d\x25\x70\x2c\x20\x3a" "\n"
,mIdentifier.c_str(),isCloned()?"\x28\x63\x6c\x6f\x6e\x65\x29":"",getLineNo(),
getSequenceNo(),this,mParent);
#endif                     
printData();if(!Mupxf)return;for(NodeMap::iterator VfDVC=mChildMap.begin();VfDVC
!=mChildMap.end();++VfDVC)VfDVC->second->print(Mupxf);if(KvFxK&&mRight){mRight->
print(Mupxf);}}void Node::printData()const{std::cerr<<
"\x62\x61\x73\x65\x20\x6e\x6f\x64\x65\x20\x68\x61\x73\x20\x6e\x6f\x20\x64\x61\x74\x61"
<<std::endl;}bool Node::read(ReaderXML*){fprintf(stderr,
"\x20\x4e\x6f\x64\x65\x3a\x3a\x72\x65\x61\x64\x3a\x20\x73\x6f\x6d\x65\x62\x6f\x64\x79\x20\x66\x6f\x72\x67\x6f\x74\x20\x74\x6f\x20\x69\x6d\x70\x6c\x65\x6d\x65\x6e\x74\x20\x61\x20\x72\x65\x61\x64\x20\x72\x6f\x75\x74\x69\x6e\x65\x21" "\n"
);return true;}void Node::parse(ParserCallback*dSmeZ,bool Mupxf){if(dSmeZ)dSmeZ
->readNode(this);if(!Mupxf)return;for(NodeMap::iterator VfDVC=mChildMap.begin();
VfDVC!=mChildMap.end();++VfDVC)VfDVC->second->parse(dSmeZ,Mupxf);if(mRight)
mRight->parse(dSmeZ,Mupxf);}void Node::prepare(bool Mupxf){static bool Y3vvh=
false;if(Y3vvh&&((mLineNo%10000)<5))fprintf(stderr,
"\x4e\x6f\x64\x65\x3a\x3a\x70\x72\x65\x70\x61\x72\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x6e\x6f\x64\x65\x20\x66\x72\x6f\x6d\x20\x6c\x69\x6e\x65\x20\x25\x64\x2c\x20\x74\x79\x70\x65\x20\x3d\x20\x25\x73" "\n"
,mLineNo,mIdentifier.c_str());calcPrepareData();if(!Mupxf)return;for(NodeMap::
iterator VfDVC=mChildMap.begin();VfDVC!=mChildMap.end();++VfDVC)VfDVC->second->
prepare(Mupxf);if(mRight)mRight->prepare(Mupxf);calcPostPrepareData();}void Node
::transform(const double&dx,const double&dy,const double&Zbgoe,const double&
nZWMW,bool Mupxf){applyTransformation(dx,dy,Zbgoe,nZWMW);if(!Mupxf)return;for(
NodeMap::iterator VfDVC=mChildMap.begin();VfDVC!=mChildMap.end();++VfDVC)VfDVC->
second->transform(dx,dy,Zbgoe,nZWMW,Mupxf);if(mRight)mRight->transform(dx,dy,
Zbgoe,nZWMW,Mupxf);}void Node::calcPrepareData(){}void Node::calcPostPrepareData
(){}void Node::applyTransformation(const double&,const double&,const double&,
const double&){}void Node::isolate(){if(mLeft)mLeft->setRight(mRight);if(mRight)
mRight->setLeft(mLeft);if(mParent&&mParent->getChild(mOpcode)==this)mParent->
setChild(mRight,mOpcode);mLeft=0;mRight=0;mParent=0;}void Node::setData(void*
data){mData=data;}void*Node::getData(){return mData;}const void*Node::getData()
const{return mData;}unsigned int Node::getDebugInfo()const{return mDebugInfo;}
unsigned int Node::getLineNo()const{return mLineNo;}unsigned int Node::
getSequenceNo()const{return(mDebugInfo&0x00ff0000)>>16;}bool Node::isCloned()
const{return(mDebugInfo&sFlagClone)!=0;}void Node::setLineNo(const unsigned int&
bBnys,const unsigned int&KRTmF){mLineNo=bBnys;mDebugInfo|=(KRTmF<<16);}const 
double&Node::getS()const{static const double trumA=-1.0;return trumA;}Node*Node
::getCopy(bool Mupxf){Node*dzamm=new Node(this);if(Mupxf)deepCopy(dzamm);return 
dzamm;}void Node::deepCopy(Node*parent){for(NodeMap::iterator VfDVC=mChildMap.
begin();VfDVC!=mChildMap.end();++VfDVC){Node*dzamm=VfDVC->second->getCopy(true);
parent->addChild(dzamm);Node*Gv01F=VfDVC->second->getRight();while(Gv01F){parent
->addChild(Gv01F->getCopy(true));Gv01F=Gv01F->getRight();}}}void Node::setLevel(
const unsigned int&M8R_J){mLevel=M8R_J;}const std::string&Node::getCDATA()const{
return mCDATA;}void Node::setCDATA(const std::string&cdata){mCDATA=cdata;}void 
Node::rotatePoint(double&x,double&y,const double&tGH3M,const double&NaBKx,const 
double&LzkrF){double dx=x-tGH3M;double dy=y-NaBKx;double tyz2F=sqrt(dx*dx+dy*dy)
;double DLJQW=atan2(dy,dx)+LzkrF;x=tGH3M+tyz2F*cos(DLJQW);y=NaBKx+tyz2F*sin(
DLJQW);}void Node::setChild(Node*rkAEh,unsigned int type){NodeMap::iterator 
VfDVC;VfDVC=mChildMap.find(type);if(VfDVC!=mChildMap.end())mChildMap.erase(VfDVC
);if(!rkAEh)return;std::pair<NodeMap::iterator,bool>QVavK=mChildMap.insert(std::
pair<unsigned int,Node*>(rkAEh->getOpcode(),rkAEh));if(!QVavK.second)std::cerr<<
"\x4e\x6f\x64\x65\x3a\x3a\x73\x65\x74\x43\x68\x69\x6c\x64\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x73\x65\x74\x20\x63\x68\x69\x6c\x64\x20\x6e\x6f\x64\x65\x21"
<<std::endl;}}
