
#include "OdrGeoParamPoly.hh"
#include "OdrGeoHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
namespace OpenDrive{GeoParamPoly::GeoParamPoly():GeoNode(
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79"),mMaxP(1.0),mUseLength(false)
,mInterpolateHeading(false),mDeltaHeading(0.0),N6H_y(0.0),mTesselation(200),
mSStep(1.0),mPStep(1.0){mOpcode=ODR_OPCODE_GEO_PARAM_POLY;mLevel=2;}GeoParamPoly
::~GeoParamPoly(){}GeoParamPoly::GeoParamPoly(GeoParamPoly*Nf2Ao):GeoNode(Nf2Ao)
{mAu=Nf2Ao->mAu;mBu=Nf2Ao->mBu;mCu=Nf2Ao->mCu;mDu=Nf2Ao->mDu;mAv=Nf2Ao->mAv;mBv=
Nf2Ao->mBv;mCv=Nf2Ao->mCv;mDv=Nf2Ao->mDv;mTesselation=Nf2Ao->getTesselation();
mMaxP=Nf2Ao->getMaxP();mUseLength=Nf2Ao->mUseLength;mInterpolateHeading=Nf2Ao->
mInterpolateHeading;mDeltaHeading=Nf2Ao->mDeltaHeading;N6H_y=Nf2Ao->N6H_y;}void 
GeoParamPoly::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x41\x75\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mAu);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x42\x75\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mBu);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x43\x75\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mCu);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x44\x75\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mDu);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x41\x76\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mAv);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x42\x76\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mBv);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x43\x76\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mCv);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x44\x76\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mDv);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x6d\x61\x78\x50\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n",
mMaxP);}bool GeoParamPoly::readIntern(ReaderXML*F3vnM){if(!F3vnM)return false;
mAu=F3vnM->getDouble("\x61\x55");mBu=F3vnM->getDouble("\x62\x55");mCu=F3vnM->
getDouble("\x63\x55");mDu=F3vnM->getDouble("\x64\x55");mAv=F3vnM->getDouble(
"\x61\x56");mBv=F3vnM->getDouble("\x62\x56");mCv=F3vnM->getDouble("\x63\x56");
mDv=F3vnM->getDouble("\x64\x56");if((mAu==0.0)&&(mBu==0.0)&&(mCu==0.0)&&(mDu==
0.0)&&(mAv==0.0)&&(mBv==0.0)&&(mCv==0.0)&&(mDv==0.0)){fprintf(stderr,
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x72\x65\x61\x64\x49\x6e\x74\x65\x72\x6e\x3a\x20\x45\x52\x52\x4f\x52\x3a\x20\x61\x6c\x6c\x20\x70\x61\x72\x61\x6d\x65\x74\x65\x72\x73\x20\x6f\x66\x20\x70\x6f\x6c\x79\x6e\x6f\x6d\x69\x61\x6c\x20\x61\x72\x65\x20\x7a\x65\x72\x6f\x2e\x20\x50\x6c\x65\x61\x73\x65\x20\x63\x68\x65\x63\x6b\x20\x73\x70\x65\x6c\x6c\x69\x6e\x67\x20\x28\x61\x55\x2c\x20\x62\x55\x2e\x2e\x29\x20\x61\x6e\x64\x20\x76\x61\x6c\x75\x65\x73" "\n"
);}mUseLength=(F3vnM->getString("\x70\x52\x61\x6e\x67\x65")==std::string(
"\x61\x72\x63\x4c\x65\x6e\x67\x74\x68"));return true;}bool GeoParamPoly::
ds2dxdydh(const double&rganP,double&dx,double&dy,double&nZWMW){double NzXSP=
ds2validDs(rganP);double p=ds2p(NzXSP);p2dxdy(p,dx,dy);if(N6H_y!=0.0){double u=
dx;double wWUNF=dy;dx=u*cos(N6H_y)-wWUNF*sin(N6H_y);dy=u*sin(N6H_y)+wWUNF*cos(
N6H_y);}nZWMW=p2dh(p)-p2dh(0.0);return true;}bool GeoParamPoly::ds2dh(const 
double&rganP,double&nZWMW){double NzXSP=ds2validDs(rganP);double p=ds2p(NzXSP);
nZWMW=p2dh(p)-p2dh(0.0);return true;}double GeoParamPoly::s2curvature(const 
double&BJBDA){return ds2curvature(BJBDA-mHdr->mS);}double GeoParamPoly::
ds2curvature(const double&rganP){double NzXSP=ds2validDs(rganP);double p=ds2p(
NzXSP);double uOXiT=getFirstDerivU(p);double DUGme=getFirstDerivV(p);double 
ns3P1=getSecondDerivU(p);double cyBha=getSecondDerivV(p);double value=sqrt(uOXiT
*uOXiT+DUGme*DUGme);return(uOXiT*cyBha-ns3P1*DUGme)/(value*value*value);}Node*
GeoParamPoly::getCopy(bool Mupxf){Node*dzamm=new GeoParamPoly(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}unsigned int GeoParamPoly::getTesselation()const{
return mTesselation;}const double&GeoParamPoly::getMaxP()const{return mMaxP;}
void GeoParamPoly::calcPostPrepareData(){static const bool PwJ1X=false;if(
mUseLength)mMaxP=mHdr->mLength;if((mAu!=0.0)||(mAv!=0.0)){double dx=mAu*cos(mHdr
->mH)-mAv*sin(mHdr->mH);double dy=mAu*sin(mHdr->mH)+mAv*cos(mHdr->mH);if(PwJ1X)
fprintf(stderr,
"\x23\x20\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x61\x55\x20\x2f\x20\x61\x56\x20\x61\x72\x65\x20\x6e\x6f\x6e\x2d\x7a\x65\x72\x6f\x3b\x20\x73\x68\x69\x66\x74\x69\x6e\x67\x20\x74\x68\x65\x20\x65\x6e\x74\x72\x79\x20\x62\x79\x20\x64\x78\x2f\x64\x79\x20\x3d\x20\x25\x2e\x33\x6c\x66\x20\x2f\x20\x25\x2e\x33\x6c\x66" "\n"
,dx,dy);mHdr->mX+=dx;mHdr->mY+=dy;mAu=0.0;mAv=0.0;}double dx;double dy;p2dxdy(
mMaxP,dx,dy);mHdr->mXEnd=mHdr->mX+dx;mHdr->mYEnd=mHdr->mY+dy;bool nY4Sv=false;
N6H_y=0.0;if(1||((fabs(p2dh(0))>1.0e-10))){mDeltaHeading=0.0;mHdr->mHEnd=mHdr->
mH+p2dh(mMaxP);N6H_y=-p2dh(0);mHdr->mH+=p2dh(0);;if(mHdr->getLeft()){GeoHeader*
XFAcn=reinterpret_cast<GeoHeader*>(mHdr->getLeft());if((1.0-cos(XFAcn->mHEnd-
mHdr->mH))>1.0e-6)fprintf(stderr,
"\x23\x20\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x64\x69\x73\x63\x72\x65\x70\x61\x6e\x63\x79\x20\x62\x65\x74\x77\x65\x65\x6e\x20\x65\x6e\x64\x20\x68\x65\x61\x64\x69\x6e\x67\x20\x6f\x66\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x20\x61\x6e\x64\x20\x73\x74\x61\x72\x74\x20\x68\x65\x61\x64\x69\x6e\x67\x20\x6f\x66\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x2c\x20\x64\x65\x6c\x74\x61\x20\x3d\x20\x25\x2e\x38\x6c\x66" "\n"
,XFAcn->getLineNo(),mHdr->getLineNo(),XFAcn->mHEnd-mHdr->mH);}if(
mInterpolateHeading){if(mHdr->getLeft()){GeoHeader*XFAcn=reinterpret_cast<
GeoHeader*>(mHdr->getLeft());GeoNode*Hdzlf=reinterpret_cast<GeoNode*>(mHdr->
getLeft()->getChild());if(Hdzlf){if(Hdzlf->getOpcode()==
ODR_OPCODE_GEO_PARAM_POLY){GeoParamPoly*NgcYA=reinterpret_cast<GeoParamPoly*>(
Hdzlf);NgcYA->mDeltaHeading=mHdr->mH-XFAcn->mHEnd;fprintf(stderr,
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x6c\x65\x66\x74\x50\x6f\x6c\x79\x2d\x3e\x6d\x44\x65\x6c\x74\x61\x48\x65\x61\x64\x69\x6e\x67\x20\x3d\x20\x25\x2e\x36\x6c\x66\x2c" "\n"
,NgcYA->mDeltaHeading);}}}}mStartCval=cos(mHdr->mH);mStartSval=sin(mHdr->mH);
nY4Sv=true;if(PwJ1X){fprintf(stderr,
"\x23\x20\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x68\x65\x61\x64\x69\x6e\x67\x20\x76\x65\x72\x69\x66\x69\x63\x61\x74\x69\x6f\x6e\x3a\x20\x63\x6f\x72\x72\x65\x63\x74\x48\x65\x61\x64\x69\x6e\x67\x20\x3d\x20\x25\x64" "\n"
,nY4Sv);mHdr->print(false,false);}}if(0){if((fabs(mHdr->mLength)>1.0e-10)&&!mHdr
->getRight()){double dx=0.0;double dy=0.0;p2dxdy(mMaxP,dx,dy);mHdr->mXEnd=mHdr->
mX+dx;mHdr->mYEnd=mHdr->mY+dy;if(nY4Sv)mHdr->mHEnd=mHdr->mH+p2dh(mMaxP);if(0&&
PwJ1X){fprintf(stderr,
"\x23\x20\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x6c\x65\x6e\x67\x74\x68\x20\x76\x65\x72\x69\x66\x69\x63\x61\x74\x69\x6f\x6e\x3a\x20\x63\x6f\x72\x72\x65\x63\x74\x48\x65\x61\x64\x69\x6e\x67\x20\x3d\x20\x25\x64" "\n"
,nY4Sv);mHdr->print(false,false);}}}if(PwJ1X){unsigned int P2bok=2000;double 
GbZcy=mMaxP/(1.0*P2bok);double BJBDA=0;double p=0;for(unsigned int i=0;i<P2bok;i
++){double OkMgB=getFirstDerivU(p);double a_acb=OkMgB*GbZcy;a_acb*=a_acb;OkMgB=
getFirstDerivV(p);double YOmxi=OkMgB*GbZcy;YOmxi*=YOmxi;BJBDA+=sqrt(a_acb+YOmxi)
;p+=GbZcy;}fprintf(stderr,
"\x23\x20\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x63\x61\x6c\x63\x50\x6f\x73\x74\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x6c\x65\x6e\x67\x74\x68\x20\x76\x65\x72\x69\x66\x69\x63\x61\x74\x69\x6f\x6e\x3a\x20\x6d\x4c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x36\x66\x2c\x20\x74\x65\x73\x74\x4c\x65\x6e\x20\x3d\x20\x25\x2e\x36\x66\x2c\x20\x74\x65\x73\x74\x4c\x65\x6e\x20\x2d\x20\x6d\x4c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x36\x66\x2c\x20\x6d\x4d\x61\x78\x50\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x6e\x6f\x50\x6f\x69\x6e\x74\x73\x20\x3d\x20\x25\x64\x2c\x20\x64\x70\x20\x3d\x20\x25\x2e\x36\x6c\x66" "\n"
,mHdr->mLength,BJBDA,BJBDA-mHdr->mLength,mMaxP,P2bok,GbZcy);}mVecPS.clear();if(
mHdr->mLength){mPStep=mMaxP/(1.0*mTesselation);double BJBDA=0;unsigned int Tqn83
=10;double xsrxe=mPStep/(1.0*Tqn83);for(unsigned int i=0;i<=mTesselation;i++){
mVecPS.push_back(BJBDA);for(unsigned int j=0;j<Tqn83;j++){double OkMgB=
getFirstDerivU(i*mPStep+j*xsrxe);double a_acb=OkMgB*xsrxe;a_acb*=a_acb;OkMgB=
getFirstDerivV(i*mPStep+j*xsrxe);double YOmxi=OkMgB*xsrxe;YOmxi*=YOmxi;BJBDA+=
sqrt(a_acb+YOmxi);}}}if(1&&PwJ1X){for(unsigned int i=0;i<mVecPS.size();i++)
fprintf(stderr,
"\x23\x20\x6d\x56\x65\x63\x50\x53\x2e\x61\x74\x28\x20\x25\x64\x20\x5b\x25\x2e\x36\x66\x5d\x20\x29\x20\x3d\x20\x25\x2e\x36\x66" "\n"
,i,i*mPStep,mVecPS.at(i));}mVecSP.clear();if(mHdr->mLength){mSStep=mHdr->mLength
/(1.0*mTesselation);double p=0.0;double BJBDA=0.0;for(unsigned int i=0;i<
mTesselation;i++){double vzWjS=i*mSStep;double GbZcy=0.01*mMaxP/mTesselation;
while((BJBDA<vzWjS)&&(fabs(BJBDA-vzWjS)>1.0e-6)){p+=GbZcy;double OkMgB=
getFirstDerivU(p);double a_acb=OkMgB*GbZcy;a_acb*=a_acb;OkMgB=getFirstDerivV(p);
double YOmxi=OkMgB*GbZcy;YOmxi*=YOmxi;BJBDA+=sqrt(a_acb+YOmxi);}mVecSP.push_back
(p);if((0&&(i==25))||(1&&PwJ1X))fprintf(stderr,
"\x23\x20\x6d\x56\x65\x63\x53\x50\x2e\x61\x74\x28\x20\x25\x69\x20\x5b\x20\x25\x2e\x36\x6c\x66\x5d\x20\x20\x29\x20\x3d\x20\x25\x2e\x36\x66\x20\x28\x74\x67\x74\x20\x25\x2e\x33\x6c\x66\x29\x2c\x20\x74\x6f\x74\x61\x6c\x20\x6c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x36\x6c\x66\x2c\x20\x6d\x53\x53\x74\x65\x70\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,i,BJBDA,mVecSP.at(i),vzWjS,mHdr->mLength,mSStep);}mVecSP.push_back(mMaxP);}if(1
&&PwJ1X){for(unsigned int i=0;i<mVecSP.size();i++)fprintf(stderr,
"\x23\x20\x6d\x56\x65\x63\x53\x50\x2e\x61\x74\x28\x20\x25\x64\x20\x5b\x25\x2e\x36\x66\x5d\x20\x29\x20\x3d\x20\x25\x2e\x36\x66" "\n"
,i,i*mSStep,mVecSP.at(i));}}double GeoParamPoly::getMaxCurvatureInRange(const 
double&yhFkv,const double&pwK_y){double l_6wW=ds2curvature(yhFkv);double JJQAW=
ds2curvature(pwK_y);return(fabs(l_6wW)>fabs(JJQAW))?l_6wW:JJQAW;}bool 
GeoParamPoly::secantAlgorithm(const double&x,const double&y,double&BJBDA,double&
fAoVN,double&piJ7w,double&ZXjjx,const double&iTz4N,const double&jWKSX,const 
double&gL0sg,const double&_UmVT,const double&Axkjj,const double&ZHaIX,const 
double&e1x0Z,const double&WRYw2){double hHPa9;double dv1Hx;double dDeNu;double 
g3CgU;double y1,ZAfxh;double CtHAd;int count=0;static const double vlkEu=1.0e-4;
dDeNu=iTz4N-x;g3CgU=jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*
g3CgU);dDeNu=Axkjj-x;g3CgU=ZHaIX-y;dv1Hx=WRYw2;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(
e1x0Z)*g3CgU);do{if(fabs(ZAfxh-y1)<vlkEu*10){double zSVmk;zSVmk=0.5*(_UmVT+WRYw2
);if(BJBDA<zSVmk){hHPa9=zSVmk;st2xyh(hHPa9,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;
g3CgU=piJ7w-y;y1=2.0*(cos(ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);dv1Hx=WRYw2;dDeNu=Axkjj
-x;g3CgU=ZHaIX-y;ZAfxh=2.0*(cos(e1x0Z)*dDeNu+sin(e1x0Z)*g3CgU);}else{dDeNu=iTz4N
-x;g3CgU=jWKSX-y;hHPa9=_UmVT;y1=2.0*(cos(gL0sg)*dDeNu+sin(gL0sg)*g3CgU);dv1Hx=
zSVmk;st2xyh(dv1Hx,0.0,fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;ZAfxh=2.0*
(cos(ZXjjx)*dDeNu+sin(ZXjjx)*g3CgU);}}BJBDA=dv1Hx-ZAfxh*(dv1Hx-hHPa9)/(ZAfxh-y1)
;if(BJBDA<_UmVT)BJBDA=_UmVT;else if(BJBDA>WRYw2)BJBDA=WRYw2;st2xyh(BJBDA,0.0,
fAoVN,piJ7w,ZXjjx);dDeNu=fAoVN-x;g3CgU=piJ7w-y;CtHAd=2*(cos(ZXjjx)*dDeNu+sin(
ZXjjx)*g3CgU);hHPa9=dv1Hx;dv1Hx=BJBDA;y1=ZAfxh;ZAfxh=CtHAd;}while(fabs(CtHAd)>
vlkEu&&(hHPa9!=dv1Hx)&&count++<100);return true;}double GeoParamPoly::ds2p(const
 double&rganP){static const bool PwJ1X=false;size_t jVPRr=mVecSP.size();if(!
jVPRr)return 0.0;unsigned int xiLx0=(int)(rganP/mSStep);double _UmVT=xiLx0*
mSStep;if(PwJ1X)fprintf(stderr,
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x64\x73\x32\x70\x3a\x20\x69\x6e\x64\x65\x78\x4c\x65\x66\x74\x20\x3d\x20\x25\x64\x2c\x20\x73\x4c\x65\x66\x74\x20\x3d\x20\x25\x2e\x36\x66\x2c\x20\x6d\x4c\x65\x6e\x67\x74\x68\x20\x3d\x20\x25\x2e\x36\x66" "\n"
,xiLx0,_UmVT,mHdr->mLength);if(xiLx0>(jVPRr-2))return mVecSP.at(jVPRr-1);if(
PwJ1X)fprintf(stderr,
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x64\x73\x32\x70\x3a\x20\x6d\x56\x65\x63\x53\x50\x2e\x61\x74\x28\x20\x69\x6e\x64\x65\x78\x4c\x65\x66\x74\x20\x29\x20\x3d\x20\x25\x2e\x33\x66" "\n"
,mVecSP.at(xiLx0));double t60J0=mVecSP.at(xiLx0)+(mVecSP.at(xiLx0+1)-mVecSP.at(
xiLx0))*(rganP-_UmVT)/mSStep;if(PwJ1X)fprintf(stderr,
"\x47\x65\x6f\x50\x61\x72\x61\x6d\x50\x6f\x6c\x79\x3a\x3a\x64\x73\x32\x70\x3a\x20\x64\x73\x20\x3d\x20\x25\x2e\x36\x6c\x66\x2c\x20\x70\x20\x3d\x20\x25\x2e\x36\x6c\x66\x2c\x20\x64\x65\x6c\x74\x61\x20\x3d\x20\x25\x2e\x36\x6c\x66" "\n"
,rganP,t60J0,rganP-t60J0);return t60J0;}double GeoParamPoly::p2ds(const double&p
){size_t jVPRr=mVecPS.size();if(!jVPRr)return 0.0;unsigned int xiLx0=(int)(p/
mPStep);double CI9Dr=xiLx0*mPStep;if(xiLx0>(jVPRr-2))return mVecPS.at(jVPRr-1);
return mVecPS.at(xiLx0)+(mVecPS.at(xiLx0+1)-mVecPS.at(xiLx0))*(p-CI9Dr)/mPStep;}
void GeoParamPoly::p2dxdy(const double&p,double&dx,double&dy){double JD4X6=mAu+
mBu*p+mCu*p*p+mDu*p*p*p;double cuRTs=mAv+mBv*p+mCv*p*p+mDv*p*p*p;dx=JD4X6*
mStartCval-cuRTs*mStartSval;dy=JD4X6*mStartSval+cuRTs*mStartCval;}double 
GeoParamPoly::p2dh(const double&p){if(!mInterpolateHeading)return atan2(
getFirstDerivV(p),getFirstDerivU(p));double LzkrF=atan2(getFirstDerivV(p),
getFirstDerivU(p));LzkrF+=p/mMaxP*mDeltaHeading;return LzkrF;}double 
GeoParamPoly::getFirstDerivU(const double&p){return mBu+2.0*mCu*p+3.0*mDu*p*p;}
double GeoParamPoly::getFirstDerivV(const double&p){return mBv+2.0*mCv*p+3.0*mDv
*p*p;}double GeoParamPoly::getSecondDerivU(const double&p){return 2.0*mCu+6.0*
mDu*p;}double GeoParamPoly::getSecondDerivV(const double&p){return 2.0*mCv+6.0*
mDv*p;}bool GeoParamPoly::linesIntersect(const double&Ujg4F,const double&cZOqC,
const double&c_TFD,const double&HaFA5,const double&G0pab,const double&VfL9M,
const double&uoAjb,const double&NEudn,double&OFCM4,double&M6WNW){double gm6Wu=(
NEudn-VfL9M)*(c_TFD-Ujg4F)-(uoAjb-G0pab)*(HaFA5-cZOqC);if(fabs(gm6Wu)<1.e-20)
return false;double VKzMa=(uoAjb-G0pab)*(cZOqC-VfL9M)-(NEudn-VfL9M)*(Ujg4F-G0pab
);VKzMa/=gm6Wu;OFCM4=Ujg4F+VKzMa*(c_TFD-Ujg4F);M6WNW=cZOqC+VKzMa*(HaFA5-cZOqC);
return true;}bool GeoParamPoly::linesIntersect(const double&Ujg4F,const double&
cZOqC,const double&yW2TH,const double&G0pab,const double&VfL9M,const double&
usHj7,double&OFCM4,double&M6WNW){return linesIntersect(Ujg4F,cZOqC,Ujg4F+cos(
yW2TH),cZOqC+sin(yW2TH),G0pab,VfL9M,G0pab+cos(usHj7),VfL9M+sin(usHj7),OFCM4,
M6WNW);}}
