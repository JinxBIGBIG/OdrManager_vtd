
#include "OdrJuncHeader.hh"
#include "OdrReaderXML.hh"
#include "OdrJuncLink.hh"
#include "OdrJuncController.hh"
#include "OdrRoadHeader.hh"
#include "OdrBbox.hh"
#include "OdrSurfaceCRG.hh"
#include <stdio.h>
namespace OpenDrive{JuncHeader::JuncHeader():Node(
"\x4a\x75\x6e\x63\x48\x65\x61\x64\x65\x72"),mBbox(0),mHasSurfaceData(false){
mOpcode=ODR_OPCODE_JUNCTION_HEADER;}JuncHeader::JuncHeader(JuncHeader*Nf2Ao):
Node(Nf2Ao),mBbox(0){mName=Nf2Ao->mName;mId=Nf2Ao->mId;mIdAsString=Nf2Ao->
mIdAsString;mIsVirtual=Nf2Ao->mIsVirtual;if(Nf2Ao->getBoundingBox())mBbox=new 
Bbox(Nf2Ao->getBoundingBox());}JuncHeader::~JuncHeader(){if(mBbox)delete mBbox;}
void JuncHeader::printData()const{fprintf(stderr,
"\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x73" "\n",
mIdAsString.c_str(),mName.c_str());fprintf(stderr,
"\x49\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n",
mIdAsString.c_str(),mId);fprintf(stderr,
"\x74\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e" "\n",mIsVirtual?
"\x76\x69\x72\x74\x75\x61\x6c":"\x64\x65\x66\x61\x75\x6c\x74");}bool JuncHeader
::read(ReaderXML*F3vnM){mName=F3vnM->getString("\x6e\x61\x6d\x65");mId=F3vnM->
getUInt("\x69\x64");mIdAsString=F3vnM->getString("\x69\x64");mIsVirtual=F3vnM->
getString("\x74\x79\x70\x65")==std::string("\x76\x69\x72\x74\x75\x61\x6c");
return true;}JuncLink*JuncHeader::getFirstLink(){return reinterpret_cast<
JuncLink*>(getChild(ODR_OPCODE_JUNCTION_LINK));}JuncController*JuncHeader::
getFirstController(){return reinterpret_cast<JuncController*>(getChild(
ODR_OPCODE_JUNCTION_CONTROL));}void JuncHeader::calcPostPrepareData(){
mHasSurfaceData=getChild(ODR_OPCODE_SURFACE_CRG)!=0;calcBoundingBox();}Bbox*
JuncHeader::getBoundingBox(){return mBbox;}void JuncHeader::getCtrAndRadius(
double&x,double&y,double&tyz2F){if(mBbox){mBbox->getCtrAndRadius(x,y,tyz2F);
return;}x=0.0;y=0.0;tyz2F=0.0;}void JuncHeader::calcBoundingBox(){if(!mBbox)
mBbox=new Bbox();JuncLink*hYfdO=getFirstLink();while(hYfdO){if(hYfdO->
mConnectingRoad)mBbox->add(hYfdO->mConnectingRoad->getBoundingBox());hYfdO=
reinterpret_cast<JuncLink*>(hYfdO->getRight());}}SurfaceCRG*JuncHeader::
getSurface(const unsigned short&TCz3u){if(!mHasSurfaceData)return 0;SurfaceCRG*
o055c=reinterpret_cast<SurfaceCRG*>(getChild(ODR_OPCODE_SURFACE_CRG));while(
o055c){if(o055c->mPurpose&TCz3u)return o055c;o055c=reinterpret_cast<SurfaceCRG*>
(o055c->getRight());}return 0;}}
