
#ifndef _OPENDRIVE_ROAD_DATA_HH
#define _OPENDRIVE_ROAD_DATA_HH
#include <map>
#include "OdrNode.hh"
#include "OdrParserCallback.hh"
namespace OpenDrive
{
  class Header;
  class Projection;
  class RoadData : public ParserCallback
  {
  private:
    static RoadData *sInstance;
    Node *mRootNode;
    typedef std::map<unsigned int, Node *> NodeMap;
    typedef std::map<std::string, Node *> NodeMapStr;
    NodeMap mFirstNodeMap;
    NodeMap mRoadHdrMap;
    NodeMapStr mRoadHdrMapStr;
    NodeMapStr
        mObjectMapStr;
    NodeMapStr mSignalMapStr;
    typedef std::pair<NodeMap::iterator, bool>
        NodeMapErr;
    unsigned int mNoModifications;
    Header *mOdrHeader;
    Projection *
        mProjection;
  public:
    explicit RoadData(Node *wrEZC = NULL);
    virtual ~RoadData();
    static RoadData *getInstance();
    void setRootNode(Node *node);
    Node *getRootNode();
    const Node
        *
        getRootNode() const;
    Node *findFirstNode(const unsigned int type);
    void
    buildIndexTable();
    virtual bool readNode(const Node *node);
    unsigned int
    getNoModifications() const;
    void incNoModifications();
    Node *getNodeFromId(const unsigned int type, unsigned int id);
    Node *getNodeFromId(const unsigned int type,
                        const std::string &iB6gF);
    unsigned int getNoRoads();
    Header *getOdrHeader();
    Projection *getProjection();
  };
} // namespace OpenDrive
#endif
