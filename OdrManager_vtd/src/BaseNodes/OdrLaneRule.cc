
#include "OdrLaneRule.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{LaneRule::LaneRule():Node("\x4c\x61\x6e\x65\x52\x75\x6c\x65"
){mOpcode=ODR_OPCODE_LANE_RULE;mLevel=3;mS=0.0;mSEnd=0.0;}LaneRule::LaneRule(
LaneRule*Nf2Ao):Node(Nf2Ao){mOffset=Nf2Ao->mOffset;mValue=Nf2Ao->mValue;}
LaneRule::~LaneRule(){}void LaneRule::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mOffset);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x76\x61\x6c\x75\x65\x3a\x20\x20\x20\x20" "\"" "\x25\x73" "\"\n"
,mValue.c_str());fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mS);fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x73\x45\x6e\x64\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66" "\n"
,mSEnd);}bool LaneRule::read(ReaderXML*F3vnM){mOffset=F3vnM->getDouble(
"\x73\x4f\x66\x66\x73\x65\x74");mValue=F3vnM->getString("\x76\x61\x6c\x75\x65");
return true;}std::string LaneRule::ds2rule(const double&rganP){double BJBDA=
rganP-mOffset;if(BJBDA<0.0)return std::string("");return mValue;}std::string 
LaneRule::s2rule(const double&BJBDA){if((BJBDA>=mS)&&(BJBDA<=mSEnd))return 
mValue;return std::string("");}Node*LaneRule::getCopy(bool Mupxf){Node*dzamm=new
 LaneRule(this);if(Mupxf)deepCopy(dzamm);return dzamm;}void LaneRule::
calcPrepareData(){LaneSection*H4prs=reinterpret_cast<LaneSection*>(getParent()->
getParent());mS=H4prs->mS+mOffset;mSEnd=H4prs->mSEnd;}}
