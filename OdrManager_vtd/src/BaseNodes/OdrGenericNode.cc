
#include "OdrGenericNode.hh"
#include <cstdio>
namespace OpenDrive{GenericNode::GenericNode(const std::string&SQVLB):Node(SQVLB
){mOpcode=ODR_OPCODE_GENERIC_NODE;}GenericNode::GenericNode(GenericNode*Nf2Ao):
Node(Nf2Ao){if(!Nf2Ao)return;mAttributesMap=Nf2Ao->mAttributesMap;}GenericNode::
~GenericNode(){mAttributesMap.clear();}void GenericNode::printData()const{std::
string DO9qH("");for(unsigned int i=0;i<mLevel;++i)DO9qH+=std::string(
"\x20\x20\x20\x20");for(ReaderXML::AttribMap::const_iterator GHusr=
mAttributesMap.begin();GHusr!=mAttributesMap.end();++GHusr){fprintf(stderr,
"\x25\x73\x20\x20\x20\x20\x25\x73\x3a\x20" "\t" "\x25\x73" "\n",DO9qH.c_str(),
GHusr->first.c_str(),GHusr->second.c_str());}}bool GenericNode::read(ReaderXML*
F3vnM){mAttributesMap=F3vnM->getAllAttributes();return true;}Node*GenericNode::
getCopy(bool Mupxf){Node*dzamm=new GenericNode(this);if(Mupxf)deepCopy(dzamm);
return dzamm;}}
