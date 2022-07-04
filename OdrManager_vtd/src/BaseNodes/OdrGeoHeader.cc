
#include "OdrGeoHeader.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include "OdrBbox.hh"
#include "OdrGeoNode.hh"
#include <stdio.h>
#include <math.h>
#include <cmath>
using namespace std;
namespace OpenDrive{GeoHeader::GeoHeader():Node(
"\x47\x65\x6f\x48\x65\x61\x64\x65\x72"),mCheckGeometry(false),mBbox(0){mOpcode=
ODR_OPCODE_GEO_HEADER;mLevel=1;mCurv=0.0;mCurvEnd=0.0;mUseCurvApprox=false;
mEnableCurvApprox=true;}GeoHeader::GeoHeader(GeoHeader*Nf2Ao):Node(Nf2Ao){mS=
Nf2Ao->mS;mX=Nf2Ao->mX;mY=Nf2Ao->mY;mH=Nf2Ao->mH;mLength=Nf2Ao->mLength;mSEnd=
Nf2Ao->mSEnd;mXEnd=Nf2Ao->mXEnd;mYEnd=Nf2Ao->mYEnd;mHEnd=Nf2Ao->mHEnd;mXCtr=
Nf2Ao->mXCtr;mYCtr=Nf2Ao->mYCtr;mCheckGeometry=Nf2Ao->mCheckGeometry;mCurv=Nf2Ao
->mCurv;mCurvEnd=Nf2Ao->mCurvEnd;mUseCurvApprox=Nf2Ao->mUseCurvApprox;if(Nf2Ao->
getBoundingBox())mBbox=new Bbox(Nf2Ao->getBoundingBox());}GeoHeader::~GeoHeader(
){if(mBbox)delete mBbox;}void GeoHeader::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x74\x68\x69\x73\x3a\x20\x20\x20\x30\x78\x25\x6c\x78" "\n",(
unsigned long long)this);fprintf(stderr,
"\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x34\x66" "\n",mS);
fprintf(stderr,
"\x20\x20\x20\x20\x78\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x34\x66" "\n",mX);
fprintf(stderr,
"\x20\x20\x20\x20\x79\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x34\x66" "\n",mY);
fprintf(stderr,
"\x20\x20\x20\x20\x48\x64\x67\x3a\x20\x20\x20\x20\x25\x2e\x34\x66" "\n",mH);
fprintf(stderr,
"\x20\x20\x20\x20\x78\x45\x6e\x64\x3a\x20\x20\x20\x25\x2e\x34\x66" "\n",mXEnd);
fprintf(stderr,
"\x20\x20\x20\x20\x79\x45\x6e\x64\x3a\x20\x20\x20\x25\x2e\x34\x66" "\n",mYEnd);
fprintf(stderr,
"\x20\x20\x20\x20\x48\x64\x67\x45\x6e\x64\x3a\x20\x25\x2e\x34\x66" "\n",mHEnd);
fprintf(stderr,
"\x20\x20\x20\x20\x4c\x65\x6e\x67\x74\x68\x3a\x20\x25\x2e\x34\x66" "\n",mLength)
;}bool GeoHeader::read(ReaderXML*F3vnM){mS=F3vnM->getDouble("\x73");mX=F3vnM->
getDouble("\x78");mY=F3vnM->getDouble("\x79");mH=F3vnM->getDouble("\x68\x64\x67"
);mLength=F3vnM->getDouble("\x6c\x65\x6e\x67\x74\x68");init();if(getLeft()){
GeoHeader*k_k98=reinterpret_cast<GeoHeader*>(getLeft());k_k98->mSEnd=mS;k_k98->
mHEnd=mH;if(k_k98->getChild()){if(k_k98->getChild()->getOpcode()==
ODR_OPCODE_GEO_LINE){while((k_k98->mHEnd-k_k98->mH)>M_PI)k_k98->mHEnd-=2.0*M_PI;
while((k_k98->mHEnd-k_k98->mH)<-M_PI)k_k98->mHEnd+=2.0*M_PI;}}k_k98->mXEnd=mX;
k_k98->mYEnd=mY;k_k98->mXCtr=0.5*(k_k98->mX+mX);k_k98->mYCtr=0.5*(k_k98->mY+mY);
}return true;}void GeoHeader::setPosEndValue(const double&ILv4i,const double&
qV6Cs,const double&xR0xN){mHEnd=xR0xN;mXEnd=ILv4i;mYEnd=qV6Cs;mXCtr=0.5*(mX+
mXEnd);mYCtr=0.5*(mY+mYEnd);}Node*GeoHeader::getCopy(bool Mupxf){Node*dzamm=new 
GeoHeader(this);if(Mupxf)deepCopy(dzamm);return dzamm;}void GeoHeader::
calcBoundingBox(const double&width){if(!mBbox)mBbox=new Bbox();GeoNode*pWj2K=
reinterpret_cast<GeoNode*>(getChild());double odw9d=1.5*width;double pxzMQ=0.0;
double kvKtF=0.0;double WDWra=0.0;double dQKLM=0.0;pWj2K->calcBoundingBox(odw9d,
pxzMQ,WDWra,kvKtF,dQKLM);mBbox->set(pxzMQ,WDWra,kvKtF,dQKLM);if(mCheckGeometry){
GeoHeader*Mwpwr=reinterpret_cast<GeoHeader*>(getRight());int I8pOy=(
reinterpret_cast<RoadHeader*>(getParent()))->mId;if(Mwpwr){double dx=Mwpwr->mX-
mXEnd;double dy=Mwpwr->mY-mYEnd;double dist=sqrt(dx*dx+dy*dy);double nZWMW=Mwpwr
->mH-mHEnd;if(dist>1.0e-4)fprintf(stderr,
"\x47\x65\x6f\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x42\x6f\x75\x6e\x64\x69\x6e\x67\x42\x6f\x78\x3a\x20\x69\x6e\x63\x6f\x6e\x73\x69\x73\x74\x65\x6e\x74\x20\x65\x6e\x64\x20\x63\x6f\x2d\x6f\x72\x64\x69\x6e\x61\x74\x65\x73\x20\x28\x63\x75\x72\x72\x65\x6e\x74\x20\x65\x6c\x65\x6d\x65\x6e\x74\x20\x2f\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x29\x20\x66\x6f\x72\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x3b\x20\x64\x69\x73\x74\x61\x6e\x63\x65\x20\x3d\x20\x25\x2e\x38\x6c\x66\x20\x28\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x73\x3d\x25\x2e\x33\x6c\x66\x29" "\n"
,mLineNo,dist,I8pOy,mS);if(cos(nZWMW)<0.999)fprintf(stderr,
"\x47\x65\x6f\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x42\x6f\x75\x6e\x64\x69\x6e\x67\x42\x6f\x78\x3a\x20\x69\x6e\x63\x6f\x6e\x73\x69\x73\x74\x65\x6e\x74\x20\x65\x6e\x64\x20\x61\x6e\x67\x6c\x65\x20\x28\x63\x75\x72\x72\x65\x6e\x74\x20\x65\x6c\x65\x6d\x65\x6e\x74\x20\x2f\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x29\x20\x66\x6f\x72\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x3b\x20\x64\x65\x6c\x74\x61\x20\x61\x6e\x67\x6c\x65\x20\x3d\x20\x25\x2e\x38\x6c\x66\x20\x28\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x73\x3d\x25\x2e\x33\x6c\x66\x29" "\n"
,mLineNo,nZWMW,I8pOy,mS);}double x;double y;double aOzQT;pWj2K->st2xyh(mSEnd,0.0
,x,y,aOzQT);double dx=mXEnd-x;double dy=mYEnd-y;double dist=sqrt(dx*dx+dy*dy);
double nZWMW=mHEnd-aOzQT;if(dist>1.0e-4)fprintf(stderr,
"\x47\x65\x6f\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x42\x6f\x75\x6e\x64\x69\x6e\x67\x42\x6f\x78\x3a\x20\x69\x6e\x63\x6f\x6e\x73\x69\x73\x74\x65\x6e\x74\x20\x65\x6e\x64\x20\x63\x6f\x2d\x6f\x72\x64\x69\x6e\x61\x74\x65\x73\x20\x28\x63\x6f\x6d\x70\x75\x74\x65\x64\x20\x76\x73\x2e\x20\x67\x69\x76\x65\x6e\x20\x69\x6e\x20\x66\x69\x6c\x65\x29\x20\x66\x6f\x72\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x3b\x20\x64\x69\x73\x74\x61\x6e\x63\x65\x20\x3d\x20\x25\x2e\x38\x6c\x66\x20\x28\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x73\x3d\x25\x2e\x33\x6c\x66\x29" "\n"
,mLineNo,dist,I8pOy,mS);if(cos(nZWMW)<0.999)fprintf(stderr,
"\x47\x65\x6f\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x42\x6f\x75\x6e\x64\x69\x6e\x67\x42\x6f\x78\x3a\x20\x69\x6e\x63\x6f\x6e\x73\x69\x73\x74\x65\x6e\x74\x20\x65\x6e\x64\x20\x61\x6e\x67\x6c\x65\x20\x28\x63\x6f\x6d\x70\x75\x74\x65\x64\x20\x76\x73\x2e\x20\x67\x69\x76\x65\x6e\x20\x69\x6e\x20\x66\x69\x6c\x65\x29\x20\x66\x6f\x72\x20\x6e\x6f\x64\x65\x20\x69\x6e\x20\x6c\x69\x6e\x65\x20\x25\x64\x3b\x20\x64\x65\x6c\x74\x61\x20\x61\x6e\x67\x6c\x65\x20\x3d\x20\x25\x2e\x38\x6c\x66\x20\x28\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x73\x3d\x25\x2e\x33\x6c\x66\x29" "\n"
,mLineNo,nZWMW,I8pOy,mS);}}Bbox*GeoHeader::getBoundingBox(){return mBbox;}bool 
GeoHeader::inBoundingBox(const double&x,const double&y){if(!mBbox)return true;
return mBbox->containsXY(x,y);}double GeoHeader::getMaxCurvatureInRange(const 
double&DvTxl,const double&LR4xi){GeoNode*pWj2K=reinterpret_cast<GeoNode*>(
getChild());if(!pWj2K)return 0.0;double xRv_o=LR4xi-mS;double rCdC0=DvTxl-mS;if(
xRv_o>mLength)xRv_o=mLength;if(rCdC0<0.0)rCdC0=0.0;return pWj2K->
getMaxCurvatureInRange(rCdC0,xRv_o);}void GeoHeader::init(){mSEnd=mS+mLength;
mHEnd=mH;mXEnd=mX;mYEnd=mY;}void GeoHeader::calcPostPrepareData(){GeoNode*pWj2K=
reinterpret_cast<GeoNode*>(getChild());if(!pWj2K)return;if(pWj2K->getOpcode()!=
ODR_OPCODE_GEO_LINE){mCurv=pWj2K->s2curvature(mS);mCurvEnd=pWj2K->s2curvature(
mSEnd);mUseCurvApprox=false;return;}GeoHeader*gP96s=reinterpret_cast<GeoHeader*>
(getLeft());GeoHeader*q1s6z=reinterpret_cast<GeoHeader*>(getRight());if(gP96s)
mCurv=gP96s->mCurvEnd;else mCurv=0.0;if(!q1s6z){mCurvEnd=0.0;return;}if(!(q1s6z
->getChild()))return;if(!mEnableCurvApprox)return;GeoNode*IVrhk=reinterpret_cast
<GeoNode*>(q1s6z->getChild());if(IVrhk->getOpcode()!=ODR_OPCODE_GEO_LINE){
mCurvEnd=IVrhk->s2curvature(mSEnd);return;}if(fabs(mLength)>1.0e-6){double jEOzZ
=q1s6z->mH-mH;while(jEOzZ>M_PI)jEOzZ-=2.0*M_PI;while(jEOzZ<-M_PI)jEOzZ+=2.0*M_PI
;mCurvEnd=jEOzZ/mLength;mHEnd=q1s6z->mH;if(fabs(mHEnd-mH)>M_PI){if(mHEnd>mH){
while((mHEnd-mH)>M_PI)mHEnd-=2.0*M_PI;}else{while((mH-mHEnd)>M_PI)mHEnd+=2.0*
M_PI;}}mUseCurvApprox=true;}}void GeoHeader::applyTransformation(const double&dx
,const double&dy,const double&,const double&nZWMW){mX+=dx;mY+=dy;mH+=nZWMW;mXEnd
+=dx;mYEnd+=dy;mHEnd+=nZWMW;mXCtr+=dx;mYCtr+=dy;rotatePoint(mX,mY,0.0,0.0,nZWMW)
;rotatePoint(mXEnd,mYEnd,0.0,0.0,nZWMW);rotatePoint(mXCtr,mYCtr,0.0,0.0,nZWMW);}
}
