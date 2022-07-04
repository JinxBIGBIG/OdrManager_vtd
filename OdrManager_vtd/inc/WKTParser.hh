
#ifndef _WKT_PARSER_HH
#define _WKT_PARSER_HH
#include "WKTNode.hh"
#include <vector>
namespace OpenDrive
{
  class WKTParser
  {
  public:
    explicit WKTParser(const std::string &
                           ddbSa);
    virtual ~WKTParser();
    virtual void parse();
    virtual void print();
    WKTNode *
    getRootNode();
  private:
    std::string mSrcString;
    WKTNode *mRootNode;
  };
} // namespace OpenDrive
#endif
