
#include <stdio.h>
#include <math.h>
#include "OdrSurfaceCRG.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadHeader.hh"
#include "OdrGeoHeader.hh"
#include "OdrGeoNode.hh"
#include "crgBaseLib.h"
#ifdef _WIN32 
#include <io.h>
#define F_OK 0 
#define access _access
#else
#include "unistd.h"
#endif
namespace OpenDrive{const unsigned int SurfaceCRG::sModeGenuine=1;const unsigned
 int SurfaceCRG::sModeAttached=2;const unsigned int SurfaceCRG::sModeAttached0=3
;const unsigned int SurfaceCRG::sModeGlobal=4;const unsigned int SurfaceCRG::
sPurposeElevation=0x0001;const unsigned int SurfaceCRG::sPurposeFriction=0x0002;
const unsigned int SurfaceCRG::sPurposeAny=0xffff;SurfaceCRG::SurfaceCRG():Node(
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47"),mMode(sModeAttached),mSOffset(0.0),
mTOffset(0.0),mZOffset(0.0),mZScale(1.0),mHOffset(0.0),mPurpose(
sPurposeElevation),mParentFileName(""),mDataSetId(-1),mCpId(-1),mGlobalSearch(
false),mInverseDirection(false),mUMin(0.0),mUMax(0.0),mVMin(0.0),mVMax(0.0){
mOpcode=ODR_OPCODE_SURFACE_CRG;mLevel=1;}SurfaceCRG::SurfaceCRG(SurfaceCRG*Nf2Ao
):Node(Nf2Ao),mDataSetId(-1),mCpId(-1){mFileName=Nf2Ao->mFileName;mS=Nf2Ao->mS;
mSEnd=Nf2Ao->mSEnd;mDir=Nf2Ao->mDir;mMode=Nf2Ao->mMode;mSOffset=Nf2Ao->mSOffset;
mTOffset=Nf2Ao->mTOffset;mZOffset=Nf2Ao->mZOffset;mZScale=Nf2Ao->mZScale;
mHOffset=Nf2Ao->mHOffset;mPurpose=Nf2Ao->mPurpose;mParentFileName=Nf2Ao->
mParentFileName;}SurfaceCRG::~SurfaceCRG(){crgDataSetRelease(mDataSetId);}void 
SurfaceCRG::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x66\x69\x6c\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73" "\n"
,mFileName.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x6d\x6f\x64\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mMode);fprintf(stderr,
"\x20\x20\x20\x20\x70\x75\x72\x70\x6f\x73\x65\x3a\x20\x20\x20\x20\x20\x25\x64" "\n"
,mPurpose);if(!mIsInJunction){fprintf(stderr,
"\x20\x20\x20\x20\x73\x53\x74\x61\x72\x74\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x73\x45\x6e\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mSEnd);fprintf(stderr,
"\x20\x20\x20\x20\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e\x3a\x20\x25\x64" "\n"
,mDir);fprintf(stderr,
"\x20\x20\x20\x20\x73\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mSOffset);fprintf(stderr,
"\x20\x20\x20\x20\x74\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mTOffset);fprintf(stderr,
"\x20\x20\x20\x20\x68\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mHOffset);}fprintf(stderr,
"\x20\x20\x20\x20\x7a\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZOffset);fprintf(stderr,
"\x20\x20\x20\x20\x7a\x53\x63\x61\x6c\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66" "\n"
,mZScale);}bool SurfaceCRG::read(ReaderXML*F3vnM){mFileName=F3vnM->getString(
"\x66\x69\x6c\x65");mS=F3vnM->getDouble("\x73\x53\x74\x61\x72\x74");mSEnd=F3vnM
->getDouble("\x73\x45\x6e\x64");mDir=F3vnM->getString(
"\x6f\x72\x69\x65\x6e\x74\x61\x74\x69\x6f\x6e")=="\x73\x61\x6d\x65"?
ODR_DIRECTION_PLUS:ODR_DIRECTION_MINUS;if(F3vnM->getString("\x6d\x6f\x64\x65")==
"\x67\x6c\x6f\x62\x61\x6c")mMode=sModeGlobal;else mMode=F3vnM->getString(
"\x6d\x6f\x64\x65")=="\x67\x65\x6e\x75\x69\x6e\x65"?sModeGenuine:((F3vnM->
getString("\x6d\x6f\x64\x65")=="\x61\x74\x74\x61\x63\x68\x65\x64\x30")?
sModeAttached0:sModeAttached);if(F3vnM->hasAttribute(
"\x73\x4f\x66\x66\x73\x65\x74"))mSOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");if(F3vnM->hasAttribute(
"\x74\x4f\x66\x66\x73\x65\x74"))mTOffset=F3vnM->getDouble(
"\x74\x4f\x66\x66\x73\x65\x74");if(F3vnM->hasAttribute(
"\x7a\x4f\x66\x66\x73\x65\x74"))mZOffset=F3vnM->getDouble(
"\x7a\x4f\x66\x66\x73\x65\x74");if(F3vnM->hasAttribute(
"\x7a\x53\x63\x61\x6c\x65"))mZScale=F3vnM->getDouble("\x7a\x53\x63\x61\x6c\x65")
;if(F3vnM->hasAttribute("\x68\x4f\x66\x66\x73\x65\x74"))mHOffset=F3vnM->
getDouble("\x68\x4f\x66\x66\x73\x65\x74");if(F3vnM->hasAttribute(
"\x70\x75\x72\x70\x6f\x73\x65")){if(F3vnM->getString(
"\x70\x75\x72\x70\x6f\x73\x65")=="\x66\x72\x69\x63\x74\x69\x6f\x6e")mPurpose=
sPurposeFriction;else if(F3vnM->getString("\x70\x75\x72\x70\x6f\x73\x65")==
"\x65\x6c\x65\x76\x61\x74\x69\x6f\x6e")mPurpose=sPurposeElevation;else mPurpose=
sPurposeAny;}if(mDir==ODR_DIRECTION_MINUS)setInverseDirection();return true;}
void SurfaceCRG::calcPrepareData(){mIsInJunction=(getParent()->getOpcode()==
ODR_OPCODE_JUNCTION_HEADER);if(mIsInJunction&&(mMode!=sModeGlobal))crgMsgPrint(
dCrgMsgLevelWarn,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x6f\x6e\x6c\x79\x20" "\"" "\x67\x6c\x6f\x62\x61\x6c" "\"" "\x20\x6d\x6f\x64\x65\x20\x69\x73\x20\x61\x6c\x6c\x6f\x77\x65\x64\x20\x69\x6e\x20\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x21\x20\x52\x65\x73\x75\x6c\x74\x73\x20\x6f\x66\x20\x75\x73\x69\x6e\x67\x20\x66\x69\x6c\x65\x20\x3c\x25\x73\x3e\x2e\x6d\x69\x67\x68\x74\x20\x62\x65\x20\x69\x6e\x76\x61\x6c\x69\x64\x2e" "\n"
,mFileName.c_str());crgMsgPrint(dCrgMsgLevelNotice,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x65\x61\x64\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x66\x69\x6c\x65\x20\x3c\x25\x73\x3e\x2e" "\n"
,mFileName.c_str());bool pRmzJ=false;RoadHeader*M8Gux=reinterpret_cast<
RoadHeader*>(getParent());if(mGlobalSearch&&M8Gux){while(M8Gux->getLeft())M8Gux=
reinterpret_cast<RoadHeader*>(M8Gux->getLeft());}while(M8Gux){SurfaceCRG*x8uV2=
reinterpret_cast<SurfaceCRG*>(M8Gux->getChild(ODR_OPCODE_SURFACE_CRG));while(
x8uV2){if(x8uV2!=this){if((x8uV2->mDataSetId>=0)&&(x8uV2->mFileName==mFileName)
&&(x8uV2->mMode==mMode)&&(x8uV2->mZScale==mZScale)){mDataSetId=x8uV2->mDataSetId
;pRmzJ=true;RoadHeader*sQZc8=reinterpret_cast<RoadHeader*>(getParent());
crgMsgPrint(dCrgMsgLevelNotice,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x75\x73\x69\x6e\x67\x20\x65\x78\x69\x73\x74\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x73\x65\x74\x20\x3c\x25\x64\x3e\x20\x66\x72\x6f\x6d\x20\x72\x6f\x61\x64\x20\x3c\x25\x64\x3e\x2e" "\n"
,sQZc8->mId,mDataSetId,M8Gux->mId);break;}}x8uV2=reinterpret_cast<SurfaceCRG*>(
x8uV2->getRight());}if(mGlobalSearch)M8Gux=reinterpret_cast<RoadHeader*>(M8Gux->
getRight());else M8Gux=reinterpret_cast<RoadHeader*>(M8Gux->getLeft());}if(!
pRmzJ){bool SACZ3=false;bool CMxBv=mFileName.length()>2;if((mFileName.c_str()[0]
!=((char)(0x55+1348-0x56a)))||(CMxBv&&(mFileName.c_str()[1]!=
((char)(0xf8d+2418-0x18c5))))){std::string LiJIF(mParentFileName);size_t b9yOf=
LiJIF.rfind(((char)(0x858+6387-0x211c)));if(b9yOf==std::string::npos)b9yOf=LiJIF
.rfind('\\');if(b9yOf!=std::string::npos){SACZ3=true;LiJIF=LiJIF.substr(0,b9yOf+
1)+mFileName;if(!access(LiJIF.c_str(),F_OK))mDataSetId=crgLoaderReadFile(LiJIF.
c_str());}}if(!SACZ3||(mDataSetId<=0)){if((mDataSetId=crgLoaderReadFile(
mFileName.c_str()))<=0){crgMsgPrint(dCrgMsgLevelFatal,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x65\x72\x72\x6f\x72\x20\x72\x65\x61\x64\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x66\x69\x6c\x65\x20\x3c\x25\x73\x3e\x2e" "\n"
,mFileName.c_str());return;}}}crgDataSetGetURange(mDataSetId,&mUMin,&mUMax);
crgDataSetGetVRange(mDataSetId,&mVMin,&mVMax);if((mCpId=crgContactPointCreate(
mDataSetId))<0){crgMsgPrint(dCrgMsgLevelFatal,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x63\x72\x65\x61\x74\x65\x20\x63\x6f\x6e\x74\x61\x63\x74\x20\x70\x6f\x69\x6e\x74\x2e" "\n"
);return;}if(0){RoadHeader*M8Gux=reinterpret_cast<RoadHeader*>(getParent());
fprintf(stderr,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x28\x29\x3a\x20\x72\x6f\x61\x64\x20\x25\x64\x20\x63\x72\x65\x61\x74\x65\x64\x20\x64\x61\x74\x61\x53\x65\x74\x20\x25\x64\x2c\x20\x63\x6f\x6e\x74\x61\x63\x74\x20\x70\x6f\x69\x6e\x74\x20\x25\x64" "\n"
,M8Gux->mId,mDataSetId,mCpId);}if(pRmzJ)return;crgDataSetModifierSetDouble(
mDataSetId,dCrgModScaleZ,mZScale);if(mMode==sModeGenuine){RoadHeader*M8Gux=
reinterpret_cast<RoadHeader*>(getParent());if(!M8Gux)return;GeoHeader*RTknE=
reinterpret_cast<GeoHeader*>(M8Gux->getFirstGeoHeader());if(!RTknE)return;while(
RTknE&&RTknE->getRight()&&(RTknE->mSEnd<mSOffset))RTknE=reinterpret_cast<
GeoHeader*>(RTknE->getRight());if(!RTknE)return;GeoNode*pWj2K=reinterpret_cast<
GeoNode*>(RTknE->getChild());if(!pWj2K)return;double x;double y;double aOzQT;
double ZXjjx;double L9KFn;pWj2K->st2xyh(mSOffset,mTOffset,x,y,aOzQT);
crgDataSetModifierSetDouble(mDataSetId,dCrgModRefPointX,x);
crgDataSetModifierSetDouble(mDataSetId,dCrgModRefPointY,y);crgEvaluv2pk(mCpId,
0.0,0.0,&ZXjjx,&L9KFn);crgDataSetModifierSetDouble(mDataSetId,
dCrgModRefLineOffsetPhi,aOzQT+mHOffset-ZXjjx);}crgDataSetModifiersPrint(
mDataSetId);crgDataSetModifiersApply(mDataSetId);}Node*SurfaceCRG::getCopy(bool 
Mupxf){Node*dzamm=new SurfaceCRG(this);if(Mupxf)deepCopy(dzamm);return dzamm;}
bool SurfaceCRG::st2zp(const double&BJBDA,const double&rkiXc,double&hkK5C,double
&p,bool BGOiw){hkK5C=0.0;p=0.0;if((BJBDA<mS)||(BJBDA>mSEnd))return false;double 
u=BJBDA-mSOffset;double wWUNF=rkiXc-mTOffset;if(mInverseDirection||(mDir==
ODR_DIRECTION_MINUS)){u=-1.0*u;wWUNF=-1.0*wWUNF;}if(!crgEvaluv2z(mCpId,u,wWUNF,&
hkK5C))return false;if(BGOiw){double s4E1B=0.0;p=s4E1B;}if(0){RoadHeader*M8Gux=
reinterpret_cast<RoadHeader*>(getParent());crgMsgPrint(dCrgMsgLevelNotice,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x73\x74\x32\x7a\x3a\x20\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x64\x61\x74\x61\x53\x65\x74\x20\x25\x64\x2c\x20\x6d\x43\x70\x49\x64\x20\x3d\x20\x25\x64\x2c\x20\x65\x76\x61\x6c\x75\x61\x74\x69\x6e\x67\x20\x61\x74\x20\x25\x2e\x33\x66\x20\x2f\x20\x25\x2e\x33\x66\x2c\x20\x7a\x20\x3d\x20\x25\x2e\x33\x66\x2c\x20\x6d\x5a\x4f\x66\x66\x73\x65\x74\x20\x3d\x20\x25\x2e\x33\x66" "\n"
,M8Gux->mId,mDataSetId,mCpId,BJBDA-mSOffset,rkiXc-mTOffset,hkK5C,mZOffset);}
hkK5C+=mZOffset;return true;}bool SurfaceCRG::xy2zp(const double&x,const double&
y,double&hkK5C,double&p,bool BGOiw){hkK5C=0.0;p=0.0;double u;double wWUNF;if(0){
for(double qO0pa=-2000.0;qO0pa<-2000.0;qO0pa+=1.0){double zmFs5=0.0;if(
crgEvalxy2uv(mCpId,qO0pa,zmFs5,&u,&wWUNF)){if(crgEvaluv2z(mCpId,u,wWUNF,&hkK5C))
{if(fabs(hkK5C)>1.0e-3)fprintf(stderr,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x3a\x78\x79\x32\x7a\x70\x3a\x20\x78\x2f\x79\x20\x3d\x20\x25\x2e\x33\x6c\x66\x20\x2f\x20\x25\x2e\x33\x6c\x66\x2c\x20\x75\x2f\x76\x20\x3d\x20\x25\x2e\x33\x6c\x66\x20\x2f\x20\x25\x2e\x33\x6c\x66\x2c\x20\x7a\x20\x3d\x20\x25\x2e\x33\x6c\x66" "\n"
,qO0pa,zmFs5,u,wWUNF,hkK5C);}}}fprintf(stderr,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x3a\x78\x79\x32\x7a\x70\x3a\x20\x53\x54\x41\x52\x54\x20\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d" "\n"
);for(double vloY2=-3000;vloY2<3000.0;vloY2+=1.0){double BkJwS=0.0;for(BkJwS=-
3000.;BkJwS<3000.0;BkJwS+=1.0){double qO0pa;double zmFs5;if(crgEvaluv2xy(mCpId,
vloY2,BkJwS,&qO0pa,&zmFs5)){if(crgEvaluv2z(mCpId,vloY2,BkJwS,&hkK5C)){if(fabs(
hkK5C)>1.0e-3){fprintf(stderr,
"\x20\x25\x2e\x33\x6c\x66\x20\x20\x25\x2e\x33\x6c\x66\x20\x20\x25\x2e\x34\x6c\x66" "\n"
,qO0pa,zmFs5,hkK5C);}}}}}fprintf(stderr,
"\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x3a\x78\x79\x32\x7a\x70\x3a\x20\x45\x4e\x44\x20\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d\x3d" "\n"
);}if(!crgEvalxy2uv(mCpId,x,y,&u,&wWUNF))return false;if(!crgEvaluv2z(mCpId,u,
wWUNF,&hkK5C))return false;if(BGOiw){double s4E1B=0.0;p=s4E1B;}hkK5C+=mZOffset;
return true;}void SurfaceCRG::enableGlobalSearch(){mGlobalSearch=true;}void 
SurfaceCRG::setInverseDirection(){mInverseDirection=true;}}
