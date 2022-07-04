
#include "OdrLaneAccess.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneAccess::LaneAccess():Node(
"\x4c\x61\x6e\x65\x41\x63\x63\x65\x73\x73"){mOpcode=ODR_OPCODE_LANE_ACCESS;
mLevel=3;}LaneAccess::LaneAccess(LaneAccess*Nf2Ao):Node(Nf2Ao){mOffset=Nf2Ao->
mOffset;mRestriction=Nf2Ao->mRestriction;mRule=Nf2Ao->mRule;}LaneAccess::~
LaneAccess(){}void LaneAccess::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOffset);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x65\x73\x74\x72\x69\x63\x74\x69\x6f\x6e\x3a\x20\x20\x20\x20\x25\x64" "\n"
,mRestriction);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x75\x6c\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64" "\n"
,mRule);}bool LaneAccess::read(ReaderXML*F3vnM){mOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");mRestriction=F3vnM->getOpcodeFromLaneAccess(
F3vnM->getString("\x72\x65\x73\x74\x72\x69\x63\x74\x69\x6f\x6e"));mRule=F3vnM->
getOpcodeFromLaneAccess(F3vnM->getString("\x72\x75\x6c\x65"));if(mRule==
ODR_LANE_ACCESS_RESTRICT_NONE)mRule=ODR_LANE_ACCESS_RESTRICT_DENY;return true;}
unsigned int LaneAccess::ds2Restriction(const double&rganP){double BJBDA=rganP-
mOffset;if(BJBDA<0.0)return ODR_LANE_ACCESS_RESTRICT_NONE;return mRestriction;}
Node*LaneAccess::getCopy(bool Mupxf){Node*dzamm=new LaneAccess(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}}
