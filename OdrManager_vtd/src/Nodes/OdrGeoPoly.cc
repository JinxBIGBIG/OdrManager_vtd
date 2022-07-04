
#include "OdrGeoPoly.hh"
#include "OdrGeoHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
namespace OpenDrive{GeoPoly::GeoPoly():GeoNode("\x47\x65\x6f\x50\x6f\x6c\x79"),
mTesselation(200),mMaxU(0.0),mSStep(1.0),mUStep(1.0){mOpcode=ODR_OPCODE_GEO_POLY
;mLevel=2;}GeoPoly::~GeoPoly(){}GeoPoly::GeoPoly(GeoPoly*Nf2Ao):GeoNode(Nf2Ao){
mA=Nf2Ao->mA;mB=Nf2Ao->mA;mC=Nf2Ao->mC;mD=Nf2Ao->mD;mTesselation=Nf2Ao->
getTesselation();mMaxU=Nf2Ao->getMaxU();}void GeoPoly::printData()const{fprintf(
stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x41\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",mA);
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x42\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",mB);
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x43\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",mC);
fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x44\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",mD);
}bool GeoPoly::readIntern(ReaderXML*F3vnM){if(F3vnM){mA=F3vnM->getDouble("\x61")
;mB=F3vnM->getDouble("\x62");mC=F3vnM->getDouble("\x63");mD=F3vnM->getDouble(
"\x64");}return true;}bool GeoPoly::ds2dxdydh(const double&rganP,double&dx,
double&dy,double&nZWMW){double NzXSP=ds2validDs(rganP);double JD4X6=ds2du(NzXSP)
;du2dxdy(JD4X6,dx,dy);nZWMW=atan(getFirstDeriv(JD4X6));return true;}bool GeoPoly
::ds2dh(const double&rganP,double&nZWMW){double NzXSP=ds2validDs(rganP);double 
JD4X6=ds2du(NzXSP);nZWMW=atan(getFirstDeriv(JD4X6));return true;}double GeoPoly
::s2curvature(const double&BJBDA){return ds2curvature(BJBDA-mHdr->mS);}double 
GeoPoly::ds2curvature(const double&rganP){double NzXSP=ds2validDs(rganP);double 
JD4X6=ds2du(NzXSP);double lgWm4=getFirstDeriv(JD4X6);double sj8U1=getSecondDeriv
(JD4X6);double value=1+lgWm4*lgWm4;value=value*value*value;return sj8U1/value;}
Node*GeoPoly::getCopy(bool Mupxf){Node*dzamm=new GeoPoly(this);if(Mupxf)deepCopy
(dzamm);return dzamm;}unsigned int GeoPoly::getTesselation()const{return 
mTesselation;}const double&GeoPoly::getMaxU()const{return mMaxU;}void GeoPoly::
calcPostPrepareData(){bool nY4Sv=false;if(fabs(mB)>1.0e-10){mHdr->mH+=atan(mB);
mStartCval=cos(mHdr->mH);mStartSval=sin(mHdr->mH);mB=0.0;nY4Sv=true;}if(mHdr->
mLength&&!mHdr->getRight()){double dx=0.0;double dy=0.0;unsigned int H5HvR=2000;
double W_9iJ=mHdr->mLength/(1.0*H5HvR);double BJBDA=0.0;mMaxU=0.0;for(unsigned 
int i=0;i<=H5HvR;i++){double OkMgB=getFirstDeriv(mMaxU);double rganP=sqrt(1+
OkMgB*OkMgB)*W_9iJ;if((BJBDA+rganP)>mHdr->mLength)W_9iJ*=0.5;else{mMaxU+=W_9iJ;
BJBDA+=rganP;}}du2dxdy(mMaxU,dx,dy);mHdr->mXEnd=mHdr->mX+dx;mHdr->mYEnd=mHdr->mY
+dy;if(nY4Sv)mHdr->mHEnd=mHdr->mH+atan(getFirstDeriv(mMaxU));}double G5eVm;
double sFQfb;linesIntersect(mHdr->mX,mHdr->mY,mHdr->mH,mHdr->mXEnd,mHdr->mYEnd,
mHdr->mH+0.5*M_PI,G5eVm,sFQfb);double dx=mHdr->mX-G5eVm;double dy=mHdr->mY-sFQfb
;mMaxU=sqrt(dx*dx+dy*dy);if(mMaxU>mHdr->mLength){fprintf(stderr,
"\x45\x52\x52\x4f\x52\x3a\x20\x47\x65\x6f\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x6d\x4c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x33\x66\x2c\x20\x6d\x4d\x61\x78\x55\x20\x3d\x20\x25\x2e\x33\x66\x2c\x20\x6d\x4d\x61\x78\x55\x20\x3c\x20\x6d\x4c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x73" "\n"
,mHdr->mLength,mMaxU,(mMaxU<mHdr->mLength)?"\x74\x72\x75\x65":
"\x66\x61\x6c\x73\x65");}unsigned int P2bok=2000;double JD4X6=mMaxU/(1.0*P2bok);
double BJBDA=0;for(unsigned int i=0;i<P2bok;i++){double OkMgB=getFirstDeriv(i*
JD4X6);BJBDA+=sqrt(1+OkMgB*OkMgB)*JD4X6;}mVecUS.clear();if(mHdr->mLength){mUStep
=mMaxU/(1.0*mTesselation);BJBDA=0;for(unsigned int i=0;i<=mTesselation;i++){
mVecUS.push_back(BJBDA);double OkMgB=getFirstDeriv(i*mUStep);BJBDA+=sqrt(1+OkMgB
*OkMgB)*mUStep;}}mVecSU.clear();if(mHdr->mLength){mSStep=mHdr->mLength/(1.0*
mTesselation);double u=0;for(unsigned int i=0;i<=mTesselation;i++){mVecSU.
push_back(u);double OkMgB=getFirstDeriv(u);double JD4X6=mSStep/sqrt(1+OkMgB*
OkMgB);BJBDA+=mSStep;u+=JD4X6;}}}double GeoPoly::getMaxCurvatureInRange(const 
double&yhFkv,const double&pwK_y){double l_6wW=ds2curvature(yhFkv);double JJQAW=
ds2curvature(pwK_y);return(fabs(l_6wW)>fabs(JJQAW))?l_6wW:JJQAW;}bool GeoPoly::
secantAlgorithm(const double&x,const double&y,double&BJBDA,double&fAoVN,double&
piJ7w,double&ZXjjx,const double&iTz4N,const double&jWKSX,const double&gL0sg,
const double&_UmVT,const double&Axkjj,const double&ZHaIX,const double&e1x0Z,
const double&WRYw2){double hHPa9;double dv1Hx;double dDeNu;double g3CgU;double 
y1,ZAfxh;double CtHAd;int count=0;static const double vlkEu=1.0e-4;dDeNu=iTz4N-x
;g3CgU=jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*g3CgU);dDeNu=
Axkjj-x;g3CgU=ZHaIX-y;dv1Hx=WRYw2;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(e1x0Z)*g3CgU);
do{if(fabs(ZAfxh-y1)<vlkEu*10){double zSVmk;zSVmk=0.5*(_UmVT+WRYw2);if(BJBDA<
zSVmk){hHPa9=zSVmk;st2xyh(hHPa9,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w
-y;y1=2.0*(cos(ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);dv1Hx=WRYw2;dDeNu=Axkjj-x;g3CgU=
ZHaIX-y;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(e1x0Z)*g3CgU);}else{dDeNu=iTz4N-x;g3CgU=
jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*g3CgU);dv1Hx=zSVmk;
st2xyh(dv1Hx,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;ZAfxh=2.0*(cos(
ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);}}BJBDA=dv1Hx-ZAfxh*(dv1Hx-hHPa9)/(ZAfxh-y1);if(
BJBDA<_UmVT)BJBDA=_UmVT;else if(BJBDA>WRYw2)BJBDA=WRYw2;st2xyh(BJBDA,0.0,fAoVN,
piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;CtHAd=2*(cos(ZXjjx)*dDeNu+sin(ZXjjx)*
g3CgU);hHPa9=dv1Hx;dv1Hx=BJBDA;y1=ZAfxh;ZAfxh=CtHAd;}while(fabs(CtHAd)>vlkEu&&(
hHPa9!=dv1Hx)&&count++<100);return true;}double GeoPoly::ds2du(const double&
rganP){size_t jVPRr=mVecSU.size();if(!jVPRr)return 0.0;unsigned int xiLx0=(int)(
rganP/mSStep);double _UmVT=xiLx0*mSStep;if(xiLx0>(jVPRr-2))return mVecSU.at(
jVPRr-1);return mVecSU.at(xiLx0)+(mVecSU.at(xiLx0+1)-mVecSU.at(xiLx0))*(rganP-
_UmVT)/mSStep;}double GeoPoly::du2ds(const double&JD4X6){size_t jVPRr=mVecUS.
size();if(!jVPRr)return 0.0;unsigned int xiLx0=(int)(JD4X6/mUStep);double twpeg=
xiLx0*mUStep;if(xiLx0>(jVPRr-2))return mVecUS.at(jVPRr-1);return mVecUS.at(xiLx0
)+(mVecUS.at(xiLx0+1)-mVecUS.at(xiLx0))*(JD4X6-twpeg)/mUStep;}void GeoPoly::
du2dxdy(const double&JD4X6,double&dx,double&dy){double cuRTs=mA+mB*JD4X6+mC*
JD4X6*JD4X6+mD*JD4X6*JD4X6*JD4X6;dx=JD4X6*mStartCval-cuRTs*mStartSval;dy=JD4X6*
mStartSval+cuRTs*mStartCval;}double GeoPoly::getFirstDeriv(const double&JD4X6){
return mB+2.0*mC*JD4X6+3.0*mD*JD4X6*JD4X6;}double GeoPoly::getSecondDeriv(const 
double&JD4X6){return 6.0*mD*JD4X6+2.0*mC;}bool GeoPoly::linesIntersect(const 
double&Ujg4F,const double&cZOqC,const double&c_TFD,const double&HaFA5,const 
double&G0pab,const double&VfL9M,const double&uoAjb,const double&NEudn,double&
OFCM4,double&M6WNW){double gm6Wu=(NEudn-VfL9M)*(c_TFD-Ujg4F)-(uoAjb-G0pab)*(
HaFA5-cZOqC);if(fabs(gm6Wu)<1.e-20)return false;double VKzMa=(uoAjb-G0pab)*(
cZOqC-VfL9M)-(NEudn-VfL9M)*(Ujg4F-G0pab);VKzMa/=gm6Wu;OFCM4=Ujg4F+VKzMa*(c_TFD-
Ujg4F);M6WNW=cZOqC+VKzMa*(HaFA5-cZOqC);return true;}bool GeoPoly::linesIntersect
(const double&Ujg4F,const double&cZOqC,const double&yW2TH,const double&G0pab,
const double&VfL9M,const double&usHj7,double&OFCM4,double&M6WNW){return 
linesIntersect(Ujg4F,cZOqC,Ujg4F+cos(yW2TH),cZOqC+sin(yW2TH),G0pab,VfL9M,G0pab+
cos(usHj7),VfL9M+sin(usHj7),OFCM4,M6WNW);}}
