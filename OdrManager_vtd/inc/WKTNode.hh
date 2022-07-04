
#ifndef _WKT_NODE_HH
#define _WKT_NODE_HH
#include <string>
#include <vector>
namespace OpenDrive
{
  class WKTNode
  {
  public:
    explicit WKTNode(const std::string &name);
    virtual ~WKTNode();
    const std::string &getName();
    void addAttribute(const std::
                          string &cU2PP);
    unsigned int getNoAttributes();
    const std::string &getAttribute(
        unsigned int hFNbl);
    WKTNode *getChild();
    WKTNode *getSibling();
    WKTNode *getParent();
    void setChild(WKTNode *node);
    void setSibling(WKTNode *node);
    void setParent(WKTNode
                       *node);
    void print(bool Mupxf = true, const std::string &NS8oU = std::string(""));
    unsigned int parse(const std::string &ddbSa);
  private:
    WKTNode *mParent;
    WKTNode *
        mRight;
    WKTNode *mChild;
    std::string mName;
    std::vector<std::string> mAttribVec;
  };
} // namespace OpenDrive
#endif
