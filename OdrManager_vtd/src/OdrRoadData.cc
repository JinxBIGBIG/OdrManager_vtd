
#include <iostream>
#include "OdrRoadData.hh"
#include "OdrRoadHeader.hh"
#include "OdrObject.hh"
#include "OdrSignal.hh"
#include "OdrHeader.hh"
#include "OdrProjection.hh"
#include "OdrGeoReference.hh"
#include <stdio.h>
namespace OpenDrive
{
  RoadData *RoadData::sInstance = NULL;
  RoadData::RoadData(Node *
                         wrEZC) : mRootNode(0), mNoModifications(0), mOdrHeader(0), mProjection(0)
  {
    if (!sInstance)
      sInstance = this;
    if (!mProjection)
      mProjection = new Projection();
    if (wrEZC)
      setRootNode(wrEZC);
  }
  RoadData::~RoadData()
  {
    if (sInstance == this)
    {
      sInstance = 0;
      if (
          mRootNode)
        delete mRootNode;
    }
    if (mProjection)
      delete mProjection;
    NodeMap::iterator
        VfDVC;
    while ((VfDVC = mFirstNodeMap.begin()) != mFirstNodeMap.end())
      mFirstNodeMap.erase(VfDVC);
  }
  RoadData *RoadData::getInstance() { return sInstance; }
  void RoadData::
      setRootNode(Node *wrEZC)
  {
    if (mRootNode && (wrEZC != mRootNode))
      delete mRootNode;
    mRootNode = wrEZC;
    if (mRootNode)
      buildIndexTable();
    mOdrHeader = 0;
  }
  Node *RoadData::
      getRootNode() { return mRootNode; }
  const Node *RoadData::getRootNode() const { return mRootNode; }
  Node *RoadData::findFirstNode(const unsigned int type)
  {
    NodeMap::
        iterator VfDVC;
    VfDVC = mFirstNodeMap.find(type);
    if (VfDVC == mFirstNodeMap.end())
      return 0;
    return VfDVC->second;
  }
  void RoadData::buildIndexTable()
  {
    if (!mRootNode)
    {
      std::cerr << "\x52\x6f\x61\x64\x44\x61\x74\x61\x3a\x3a\x62\x75\x69\x6c\x64\x49\x6e\x64\x65\x78\x54\x61\x62\x6c\x65\x3a\x20\x6e\x6f\x20\x72\x6f\x6f\x74\x20\x6e\x6f\x64\x65\x20\x64\x65\x66\x69\x6e\x65\x64"
                << std::endl;
      return;
    }
    mFirstNodeMap.clear();
    mRootNode->parse(this);
    mRoadHdrMap.clear();
    mRoadHdrMapStr.clear();
    mObjectMapStr.clear();
    mSignalMapStr.clear();
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(findFirstNode(
        ODR_OPCODE_ROAD_HEADER));
    while (REKut)
    {
      NodeMapErr QVavK = mRoadHdrMap.insert(std::
                                                pair<unsigned int, Node *>(REKut->mId, REKut));
      if (!QVavK.second)
        std::cerr << "\x52\x6f\x61\x64\x44\x61\x74\x61\x3a\x3a\x62\x75\x69\x6c\x64\x49\x6e\x64\x65\x78\x54\x61\x62\x6c\x65\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x61\x64\x64\x20\x69\x6e\x64\x65\x78\x20\x66\x6f\x72\x20\x72\x6f\x61\x64\x20\x6e\x6f\x2e\x20"
                  << REKut->mId << "\x2e\x20\x50\x6c\x65\x61\x73\x65\x20\x63\x68\x65\x63\x6b\x20\x66\x6f\x72\x20\x49\x44\x3f"
                  << std::endl;
      mRoadHdrMapStr.insert(std::pair<std::string, Node *>(REKut->mIdAsString, REKut));
      Object *QAAX5 = REKut->getFirstObject();
      while (QAAX5)
      {
        mObjectMapStr.insert(std::pair<std::string, Node *>(QAAX5->mIdAsString, QAAX5));
        QAAX5 = reinterpret_cast<Object *>(QAAX5->getRight());
      }
      Signal *kyJQa = REKut->getFirstSignal();
      while (kyJQa)
      {
        mSignalMapStr.insert(std::pair<std::string, Node *>(
            kyJQa->mIdAsString, kyJQa));
        kyJQa = reinterpret_cast<Signal *>(kyJQa->getRight());
      }
      REKut = reinterpret_cast<RoadHeader *>(REKut->getRight());
    }
    GeoReference *hjZsz =
        reinterpret_cast<GeoReference *>(findFirstNode(ODR_OPCODE_GEO_REFERENCE));
    if (
        hjZsz)
    {
      hjZsz->mDataString = hjZsz->getCDATA();
      if (mProjection)
      {
        if (hjZsz->getCDATA()
                .length())
          mProjection->setOGCWKT(hjZsz->getCDATA());
      }
    }
  }
  bool RoadData::readNode(
      const Node *node)
  {
    NodeMap::iterator VfDVC;
    VfDVC = mFirstNodeMap.find(node->getOpcode());
    if (VfDVC != mFirstNodeMap.end())
      return true;
    NodeMapErr QVavK =
        mFirstNodeMap.insert(std::pair<unsigned int, Node *>(node->getOpcode(), const_cast<
                                                                                    Node *>(node)));
    if (QVavK.second)
      return true;
    std::cerr << "\x52\x6f\x61\x64\x44\x61\x74\x61\x3a\x3a\x72\x65\x61\x64\x4e\x6f\x64\x65\x3a\x20\x65\x72\x72\x6f\x72\x20\x61\x64\x64\x69\x6e\x67\x20\x6e\x6f\x64\x65\x20\x6f\x66\x20\x74\x79\x70\x65\x20"
              << node->getOpcode() << "\x2e" << std::endl;
    return false;
  }
  unsigned int RoadData::
      getNoModifications() const { return mNoModifications; }
  void RoadData::
      incNoModifications() { mNoModifications++; }
  Node *RoadData::getNodeFromId(const unsigned int type, unsigned int id)
  {
    if (type != ODR_OPCODE_ROAD_HEADER)
      return 0;
    NodeMap::iterator VfDVC = mRoadHdrMap.find(id);
    if (VfDVC == mRoadHdrMap.end())
      return 0;
    return VfDVC->second;
  }
  Node *RoadData::getNodeFromId(const unsigned int type,
                                const std::string &iB6gF)
  {
    if (type == ODR_OPCODE_ROAD_HEADER)
    {
      NodeMapStr::iterator
          VfDVC = mRoadHdrMapStr.find(iB6gF);
      if (VfDVC == mRoadHdrMapStr.end())
        return 0;
      return VfDVC->second;
    }
    else if (type == ODR_OPCODE_OBJECT)
    {
      NodeMapStr::iterator VfDVC =
          mObjectMapStr.find(iB6gF);
      if (VfDVC == mObjectMapStr.end())
        return 0;
      return VfDVC->second;
    }
    else if (type == ODR_OPCODE_SIGNAL)
    {
      NodeMapStr::iterator VfDVC =
          mSignalMapStr.find(iB6gF);
      if (VfDVC == mSignalMapStr.end())
        return 0;
      return VfDVC->second;
    }
    return 0;
  }
  unsigned int RoadData::getNoRoads()
  {
    return mRoadHdrMap.size();
  }
  Header *RoadData::getOdrHeader()
  {
    if (!mOdrHeader)
      mOdrHeader = (Header *)
          findFirstNode(ODR_OPCODE_HEADER);
    return mOdrHeader;
  }
  Projection *RoadData::
      getProjection() { return mProjection; }
} // namespace OpenDrive
