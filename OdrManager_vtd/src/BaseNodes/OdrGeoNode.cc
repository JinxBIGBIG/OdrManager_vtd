
#include "OdrGeoHeader.hh"
#include "OdrGeoNode.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
#define l0CGi  -1.e-10        
#define j6thT 1.e-4         
namespace OpenDrive{GeoNode::GeoNode(const std::string&name):Node(name),mHdr(0),
mDs0(0.0){}GeoNode::GeoNode(GeoNode*Nf2Ao):Node(Nf2Ao),mHdr(0),mDs0(0.0){
mStartCval=Nf2Ao->mStartCval;mStartSval=Nf2Ao->mStartSval;mDs=Nf2Ao->mDs;}
GeoNode::~GeoNode(){}void GeoNode::printData()const{
#if 1
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x74\x68\x69\x73\x3a\x20\x20\x20\x30\x78\x25\x6c\x78" "\n"
,(unsigned long long)(this));fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x48\x64\x72\x3a\x20\x20\x20\x30\x78\x25\x6c\x78" "\n"
,(unsigned long long)(mHdr));
#endif
}bool GeoNode::read(ReaderXML*F3vnM){mHdr=reinterpret_cast<GeoHeader*>(getParent
());if(mHdr){mStartCval=cos(mHdr->mH);mStartSval=sin(mHdr->mH);}if(!readIntern(
F3vnM))return false;double x;double y;double aOzQT;if(mHdr){st2xyh(mHdr->mSEnd,
0.0,x,y,aOzQT);mHdr->setPosEndValue(x,y,aOzQT);}return true;}bool GeoNode::
st2xyh(const double&BJBDA,const double&rkiXc,double&x,double&y,double&aOzQT){
static const bool PwJ1X=false;double dx;double dy;double nZWMW;double rganP=
BJBDA-mHdr->mS;if(!ds2dxdydh(rganP,dx,dy,nZWMW))return false;aOzQT=mHdr->mH+
nZWMW;x=mHdr->mX+dx-rkiXc*sin(aOzQT);y=mHdr->mY+dy+rkiXc*cos(aOzQT);if(PwJ1X)
fprintf(stderr,
"\x47\x65\x6f\x4e\x6f\x64\x65\x3a\x3a\x73\x74\x32\x78\x79\x68\x3a\x20\x73\x2f\x74\x20\x3d\x20\x25\x2e\x33\x6c\x66\x20\x2f\x20\x25\x2e\x33\x6c\x66\x2c\x20\x78\x2f\x79\x20\x3d\x20\x25\x2e\x33\x6c\x66\x20\x2f\x20\x25\x2e\x33\x6c\x66\x2c\x20\x68\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,BJBDA,rkiXc,x,y,aOzQT);return true;}bool GeoNode::s2h(const double&BJBDA,double
&aOzQT){double nZWMW;if(!ds2dh(BJBDA-mHdr->mS,nZWMW))return false;aOzQT=mHdr->mH
+nZWMW;return true;}bool GeoNode::ds2dxdydh(const double&,double&dx,double&dy,
double&nZWMW){dx=0.0;dy=0.0;nZWMW=0.0;return true;}bool GeoNode::ds2dh(const 
double&,double&nZWMW){nZWMW=0.0;return true;}double GeoNode::s2curvature(const 
double&BJBDA){if(!getParent())return 0.0;if(getParent()->getOpcode()!=
ODR_OPCODE_GEO_HEADER)return 0.0;GeoHeader*RTknE=reinterpret_cast<GeoHeader*>(
getParent());if(!(RTknE->mUseCurvApprox))return 0.0;if(RTknE->mLength<1.0e-2)
return RTknE->mCurv;return RTknE->mCurv+((RTknE->mCurvEnd-RTknE->mCurv)/RTknE->
mLength*(BJBDA-RTknE->mS));}bool GeoNode::containsPos(const double&x,const 
double&y){return pointIsInSection(x,y,mHdr->mX,mHdr->mY,mHdr->mH,mHdr->mS,mHdr->
mXEnd,mHdr->mYEnd,mHdr->mHEnd,mHdr->mSEnd,sMaxSearchLevel);}void GeoNode::
calcPrepareData(){mHdr=reinterpret_cast<GeoHeader*>(getParent());}void GeoNode::
calcBoundingBox(const double&wYv5i,double&pxzMQ,double&WDWra,double&kvKtF,double
&dQKLM){double yhFkv=1.0;double pwK_y=10.0;double rganP=mHdr->mLength*0.01;
double x=0.0;double y=0.0;double aOzQT=0.0;double BJBDA=mHdr->mS;bool Xvqya=true
;if(rganP<yhFkv)rganP=yhFkv;if(rganP>pwK_y)rganP=pwK_y;while(BJBDA<=mHdr->mSEnd)
{st2xyh(BJBDA,wYv5i,x,y,aOzQT);if(Xvqya){pxzMQ=x;WDWra=y;kvKtF=x;dQKLM=y;Xvqya=
false;}else{pxzMQ=x<pxzMQ?x:pxzMQ;WDWra=y<WDWra?y:WDWra;kvKtF=x>kvKtF?x:kvKtF;
dQKLM=y>dQKLM?y:dQKLM;}st2xyh(BJBDA,-wYv5i,x,y,aOzQT);pxzMQ=x<pxzMQ?x:pxzMQ;
WDWra=y<WDWra?y:WDWra;kvKtF=x>kvKtF?x:kvKtF;dQKLM=y>dQKLM?y:dQKLM;if(BJBDA<mHdr
->mSEnd){BJBDA+=rganP;if(BJBDA>mHdr->mSEnd)BJBDA=mHdr->mSEnd;}else break;}}void 
GeoNode::setHeader(GeoHeader*hdr){if(!hdr)return;mHdr=hdr;}double GeoNode::
getMaxCurvatureInRange(const double&,const double&){return 0.0;}const double&
GeoNode::ds2validDs(const double&rganP){if(rganP<0.0)return mDs0;if(rganP>mHdr->
mLength)return mHdr->mLength;return rganP;}bool GeoNode::pointIsInSection(const 
double&x,const double&y,const double&iTz4N,const double&jWKSX,const double&gL0sg
,const double&_UmVT,const double&Axkjj,const double&ZHaIX,const double&e1x0Z,
const double&WRYw2,unsigned int M8R_J){double jB3MZ;double xIwjk;double aAXU1;
double zSVmk;if(!M8R_J)return false;if(pointIsBetween(x,y,iTz4N,jWKSX,gL0sg,
Axkjj,ZHaIX,e1x0Z))return true;if(M8R_J==1)return false;zSVmk=0.5*(WRYw2+_UmVT);
if(!st2xyh(zSVmk,0.0,jB3MZ,xIwjk,aAXU1))return false;if(pointIsInSection(x,y,
iTz4N,jWKSX,gL0sg,_UmVT,jB3MZ,xIwjk,aAXU1,zSVmk,M8R_J-1))return true;return 
pointIsInSection(x,y,jB3MZ,xIwjk,aAXU1,zSVmk,Axkjj,ZHaIX,e1x0Z,WRYw2,M8R_J-1);}
bool GeoNode::pointIsBetween(const double&x,const double&y,const double&iTz4N,
const double&jWKSX,const double&gL0sg,const double&Axkjj,const double&ZHaIX,
const double&e1x0Z){static const bool PwJ1X=false;double War30[2];double ZVdfk[2
];War30[0]=cos(gL0sg);War30[1]=sin(gL0sg);ZVdfk[0]=x-iTz4N;ZVdfk[1]=y-jWKSX;
double SBnCN=War30[0]*ZVdfk[0]+War30[1]*ZVdfk[1];if(SBnCN<l0CGi)return false;
War30[0]=-cos(e1x0Z);War30[1]=-sin(e1x0Z);ZVdfk[0]=x-Axkjj;ZVdfk[1]=y-ZHaIX;if(
PwJ1X){double CovTQ=War30[0]*ZVdfk[0]+War30[1]*ZVdfk[1];fprintf(stderr,
"\x47\x65\x6f\x4e\x6f\x64\x65\x3a\x3a\x70\x6f\x69\x6e\x74\x49\x73\x42\x65\x74\x77\x65\x65\x6e\x3a\x20\x64\x70\x31\x20\x3d\x20\x25\x2e\x38\x66\x2c\x20\x64\x70\x32\x20\x3d\x20\x25\x2e\x38\x66\x2c\x20\x78\x52\x69\x67\x68\x74\x20\x3d\x20\x25\x2e\x31\x36\x65\x2c\x20\x79\x52\x69\x67\x68\x74\x20\x3d\x20\x25\x2e\x31\x36\x65" "\n"
,SBnCN,CovTQ,Axkjj,ZHaIX);}SBnCN=War30[0]*ZVdfk[0]+War30[1]*ZVdfk[1];return 
SBnCN>=l0CGi;}bool GeoNode::xy2st(const double&x,const double&y,double&BJBDA,
double&rkiXc){double fAoVN;double piJ7w;double ZXjjx;double h4F_Q;BJBDA=0.0;
rkiXc=0.0;secantAlgorithm(x,y,h4F_Q,fAoVN,piJ7w,ZXjjx,mHdr->mX,mHdr->mY,mHdr->mH
,mHdr->mS,mHdr->mXEnd,mHdr->mYEnd,mHdr->mHEnd,mHdr->mSEnd);BJBDA=h4F_Q;double dx
=x-fAoVN;double dy=y-piJ7w;rkiXc=sqrt(dx*dx+dy*dy);rkiXc*=getSideOfPoint(fAoVN,
piJ7w,ZXjjx,x,y);return true;}bool GeoNode::secantAlgorithm(const double&x,const
 double&y,double&BJBDA,double&fAoVN,double&piJ7w,double&ZXjjx,const double&iTz4N
,const double&jWKSX,const double&gL0sg,const double&_UmVT,const double&Axkjj,
const double&ZHaIX,const double&e1x0Z,const double&WRYw2){double hHPa9;double 
dv1Hx;double dDeNu;double g3CgU;double y1,ZAfxh;double CtHAd;int count=0;dDeNu=
iTz4N-x;g3CgU=jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*g3CgU);
dDeNu=Axkjj-x;g3CgU=ZHaIX-y;dv1Hx=WRYw2;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(e1x0Z)*
g3CgU);do{if(fabs(ZAfxh-y1)<j6thT*10){double zSVmk;zSVmk=0.5*(_UmVT+WRYw2);if(
BJBDA<zSVmk){hHPa9=zSVmk;st2xyh(hHPa9,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU
=piJ7w-y;y1=2.0*(cos(ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);dv1Hx=WRYw2;dDeNu=Axkjj-x;
g3CgU=ZHaIX-y;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(e1x0Z)*g3CgU);}else{dDeNu=iTz4N-x;
g3CgU=jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*g3CgU);dv1Hx=zSVmk
;st2xyh(dv1Hx,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;ZAfxh=2.0*(cos(
ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);}}BJBDA=dv1Hx-ZAfxh*(dv1Hx-hHPa9)/(ZAfxh-y1);if(
BJBDA<_UmVT)BJBDA=_UmVT;else if(BJBDA>WRYw2)BJBDA=WRYw2;st2xyh(BJBDA,0.0,fAoVN,
piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;CtHAd=2*(cos(ZXjjx)*dDeNu+sin(ZXjjx)*
g3CgU);hHPa9=dv1Hx;dv1Hx=BJBDA;y1=ZAfxh;ZAfxh=CtHAd;}while(fabs(CtHAd)>j6thT&&(
hHPa9!=dv1Hx)&&count++<100);return true;}double GeoNode::getSideOfPoint(const 
double&v6YRx,const double&p8tW0,const double&ZXjjx,const double&x,const double&y
){double dx;double dy;double KsY2n;double waOPW;double GbZcy;dx=x-v6YRx;dy=y-
p8tW0;waOPW=cos(ZXjjx);KsY2n=-sin(ZXjjx);GbZcy=dx*KsY2n+dy*waOPW;return(GbZcy>
0.0)?1.0:-1.0;}}
