
#include "OdrJuncGroup.hh"
#include "OdrJuncRef.hh"
#include "OdrJuncHeader.hh"
#include "OdrReaderXML.hh"
#include "OdrBbox.hh"
#include <stdio.h>
namespace OpenDrive{JuncGroup::JuncGroup():Node(
"\x4a\x75\x6e\x63\x47\x72\x6f\x75\x70"),mBbox(0){mOpcode=
ODR_OPCODE_JUNCTION_GROUP;}JuncGroup::JuncGroup(JuncGroup*Nf2Ao):Node(Nf2Ao),
mBbox(0){mName=Nf2Ao->mName;mId=Nf2Ao->mId;mIdAsString=Nf2Ao->mIdAsString;mName=
Nf2Ao->mType;if(Nf2Ao->getBoundingBox())mBbox=new Bbox(Nf2Ao->getBoundingBox());
}JuncGroup::~JuncGroup(){if(mBbox)delete mBbox;}void JuncGroup::printData()const
{fprintf(stderr,"\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x73" "\n",
mName.c_str());fprintf(stderr,
"\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64" "\n",
mIdAsString.c_str(),mId);fprintf(stderr,
"\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x73" "\n",mType.c_str());}bool
 JuncGroup::read(ReaderXML*F3vnM){mName=F3vnM->getString("\x6e\x61\x6d\x65");mId
=F3vnM->getUInt("\x69\x64");mIdAsString=F3vnM->getString("\x69\x64");mType=F3vnM
->getString("\x74\x79\x70\x65");return true;}JuncRef*JuncGroup::getFirstJuncRef(
){return reinterpret_cast<JuncRef*>(getChild(ODR_OPCODE_JUNCTION_REFERENCE));}
void JuncGroup::calcPostPrepareData(){calcBoundingBox();}Bbox*JuncGroup::
getBoundingBox(){return mBbox;}void JuncGroup::getCtrAndRadius(double&x,double&y
,double&tyz2F){if(mBbox){mBbox->getCtrAndRadius(x,y,tyz2F);return;}x=0.0;y=0.0;
tyz2F=0.0;}void JuncGroup::calcBoundingBox(){if(!mBbox)mBbox=new Bbox();JuncRef*
ref=getFirstJuncRef();while(ref){if(ref->mJuncHeader)mBbox->add(ref->mJuncHeader
->getBoundingBox());ref=reinterpret_cast<JuncRef*>(ref->getRight());}}}
