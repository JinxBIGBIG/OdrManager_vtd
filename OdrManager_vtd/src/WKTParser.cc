
#include "WKTParser.hh"
#include <stdio.h>
namespace OpenDrive{WKTParser::WKTParser(const std::string&ddbSa):mRootNode(0){
mSrcString=ddbSa;}WKTParser::~WKTParser(){}void WKTParser::parse(){mRootNode=new
 WKTNode("\x52\x4f\x4f\x54");mRootNode->parse(mSrcString);}void WKTParser::print
(){if(!mRootNode)fprintf(stderr,
"\x57\x4b\x54\x50\x61\x72\x73\x65\x72\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x6e\x6f\x20\x64\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x21" "\n"
);mRootNode->print();}WKTNode*WKTParser::getRootNode(){return mRootNode;}}
