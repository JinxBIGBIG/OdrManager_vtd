
#include "OdrObjectMaterial.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{ObjectMaterial::ObjectMaterial():Node(
"\x4f\x62\x6a\x65\x63\x74\x4d\x61\x74\x65\x72\x69\x61\x6c"){mOpcode=
ODR_OPCODE_OBJECT_MATERIAL;mLevel=3;}ObjectMaterial::ObjectMaterial(
ObjectMaterial*Nf2Ao):Node(Nf2Ao){mFriction=Nf2Ao->mFriction;mRoughness=Nf2Ao->
mRoughness;mSurface=Nf2Ao->mSurface;}ObjectMaterial::~ObjectMaterial(){}void 
ObjectMaterial::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x46\x72\x69\x63\x74\x69\x6f\x6e\x3a\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mFriction);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x75\x67\x68\x6e\x65\x73\x73\x3a\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mRoughness);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x53\x75\x72\x66\x61\x63\x65\x3a\x20\x20\x20\x25\x73" "\n"
,mSurface.c_str());}bool ObjectMaterial::read(ReaderXML*F3vnM){mFriction=F3vnM->
getDouble("\x66\x72\x69\x63\x74\x69\x6f\x6e");mRoughness=F3vnM->getDouble(
"\x72\x6f\x75\x67\x68\x6e\x65\x73\x73");mSurface=F3vnM->getString(
"\x73\x75\x72\x66\x61\x63\x65");return true;}Node*ObjectMaterial::getCopy(bool 
Mupxf){Node*dzamm=new ObjectMaterial(this);if(Mupxf)deepCopy(dzamm);return dzamm
;}}
