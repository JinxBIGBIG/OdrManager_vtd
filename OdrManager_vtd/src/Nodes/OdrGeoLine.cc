
#include "OdrGeoLine.hh"
#include "OdrGeoHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{GeoLine::GeoLine():GeoNode("\x47\x65\x6f\x4c\x69\x6e\x65"){
mOpcode=ODR_OPCODE_GEO_LINE;mLevel=2;}GeoLine::~GeoLine(){}GeoLine::GeoLine(
GeoLine*Nf2Ao):GeoNode(Nf2Ao){}void GeoLine::printData()const{GeoNode::printData
();fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6e\x6f\x20\x66\x75\x72\x74\x68\x65\x72\x20\x64\x61\x74\x61" "\n"
);}bool GeoLine::readIntern(ReaderXML*){return true;}bool GeoLine::ds2dxdydh(
const double&rganP,double&dx,double&dy,double&nZWMW){double NzXSP=ds2validDs(
rganP);dx=NzXSP*mStartCval;dy=NzXSP*mStartSval;if((mHdr->mH!=mHdr->mHEnd)&&(fabs
(mHdr->mLength)>1.0e-5))nZWMW=(mHdr->mHEnd-mHdr->mH)/mHdr->mLength*NzXSP;else 
nZWMW=0.0;return true;}Node*GeoLine::getCopy(bool Mupxf){Node*dzamm=new GeoLine(
this);if(Mupxf)deepCopy(dzamm);return dzamm;}bool GeoLine::xy2st(const double&x,
const double&y,double&BJBDA,double&rkiXc){double dx=x-mHdr->mX;double dy=y-mHdr
->mY;double tyz2F=sqrt(dx*dx+dy*dy);double CbIFu=1.0;if(fabs(tyz2F)>1.0e-6){dx/=
tyz2F;dy/=tyz2F;CbIFu=dx*cos(mHdr->mH)+dy*sin(mHdr->mH);}if(fabs(CbIFu)>1.0)
CbIFu=CbIFu>0.0?1.0:-1.0;double KV0gK=acos(CbIFu);BJBDA=mHdr->mS+CbIFu*tyz2F;if(
fabs(BJBDA)<1.0e-6)BJBDA=0.0;if(fabs(BJBDA-mHdr->mSEnd)<1.0e-6)BJBDA=mHdr->mSEnd
;if(BJBDA<mHdr->mS||BJBDA>mHdr->mSEnd){if((BJBDA>mHdr->mSEnd)&&(mHdr->mH!=mHdr->
mHEnd)){double dx=x-mHdr->mXEnd;double dy=y-mHdr->mYEnd;double tyz2F=sqrt(dx*dx+
dy*dy);double CbIFu=dx*cos(mHdr->mHEnd)+dy*sin(mHdr->mHEnd);if(fabs(CbIFu)>1.0)
CbIFu=CbIFu>0.0?1.0:-1.0;KV0gK=acos(CbIFu);BJBDA=mHdr->mSEnd+CbIFu*tyz2F;if(
BJBDA>mHdr->mSEnd){return false;}}else{return false;}}rkiXc=tyz2F*sin(KV0gK);if(
fabs(rkiXc)<1.0e-6)rkiXc=0.0;if((dx*cos(mHdr->mH+M_PI_2)+dy*sin(mHdr->mH+M_PI_2)
)<0.0)rkiXc*=-1.0;return true;}bool GeoLine::containsPos(const double&x,const 
double&y){return pointIsBetween(x,y,mHdr->mX,mHdr->mY,mHdr->mH,mHdr->mXEnd,mHdr
->mYEnd,mHdr->mHEnd);}void GeoLine::calcBoundingBox(const double&wYv5i,double&
pxzMQ,double&WDWra,double&kvKtF,double&dQKLM){double x;double y;double aOzQT;
st2xyh(mHdr->mS,wYv5i,pxzMQ,WDWra,aOzQT);kvKtF=pxzMQ;dQKLM=WDWra;st2xyh(mHdr->mS
,-wYv5i,x,y,aOzQT);pxzMQ=x<pxzMQ?x:pxzMQ;WDWra=y<WDWra?y:WDWra;kvKtF=x>kvKtF?x:
kvKtF;dQKLM=y>dQKLM?y:dQKLM;st2xyh(mHdr->mSEnd,wYv5i,x,y,aOzQT);pxzMQ=x<pxzMQ?x:
pxzMQ;WDWra=y<WDWra?y:WDWra;kvKtF=x>kvKtF?x:kvKtF;dQKLM=y>dQKLM?y:dQKLM;st2xyh(
mHdr->mSEnd,-wYv5i,x,y,aOzQT);pxzMQ=x<pxzMQ?x:pxzMQ;WDWra=y<WDWra?y:WDWra;kvKtF=
x>kvKtF?x:kvKtF;dQKLM=y>dQKLM?y:dQKLM;}}
