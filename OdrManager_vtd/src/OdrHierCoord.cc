
#include <stdio.h>
#include "OdrNode.hh"
#include "OdrHierCoord.hh"
namespace OpenDrive
{
  HierCoord::HierCoord() : mLocked(false) { init(); }
  HierCoord::~HierCoord() {}
  void HierCoord::init() {}
  void HierCoord::operator=(const HierCoord &
                                Nf2Ao)
  {
    mLocked = Nf2Ao.mLocked;
    mNodeAccessVec = Nf2Ao.mNodeAccessVec;
    mNodeAccessVecBackup = Nf2Ao.mNodeAccessVecBackup;
  }
  void HierCoord::print() const
  {
    for (unsigned int i = 0; i < mNodeAccessVec.size(); i++)
      if (mNodeAccessVec[i])
        fprintf(
            stderr,
            "\x48\x69\x65\x72\x43\x6f\x6f\x72\x64\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x6f\x70\x63\x6f\x64\x65\x20\x3d\x20\x25\x64\x2c\x20\x70\x74\x72\x20\x3d\x20\x25\x70"
            "\n",
            i, mNodeAccessVec[i]);
  }
  bool HierCoord::keepNode(Node *node)
  {
    if (mLocked)
      return false;
    checkSize(node->getOpcode());
    if (mNodeAccessVec[node->getOpcode()] == node)
      return false;
    clearNode(node->getOpcode());
    mNodeAccessVec[node->getOpcode()] = node;
    return false;
  }
  bool HierCoord::clearNode(const unsigned int type)
  {
    if (mLocked)
      return false;
    checkSize(type);
    if (mNodeAccessVec[type])
    {
      mNodeAccessVec[type] = 0;
      switch (type)
      {
      case ODR_OPCODE_ROAD_HEADER:
        clearNode(ODR_OPCODE_ROAD_LINK);
        clearNode(ODR_OPCODE_ROAD_TYPE);
        clearNode(ODR_OPCODE_GEO_HEADER);
        clearNode(
            ODR_OPCODE_ELEVATION);
        clearNode(ODR_OPCODE_CROSSFALL);
        clearNode(
            ODR_OPCODE_SUPERELEVATION);
        clearNode(ODR_OPCODE_LANE_SECTION);
        clearNode(
            ODR_OPCODE_OBJECT);
        clearNode(ODR_OPCODE_SIGNAL);
        clearNode(ODR_OPCODE_SURFACE_CRG);
        break;
      case ODR_OPCODE_GEO_HEADER:
        clearNode(ODR_OPCODE_GEO_LINE);
        clearNode(
            ODR_OPCODE_GEO_SPIRAL);
        clearNode(ODR_OPCODE_GEO_ARC);
        break;
      case ODR_OPCODE_LANE_SECTION:
        clearNode(ODR_OPCODE_LANE);
        break;
      case ODR_OPCODE_LANE:
        clearNode(ODR_OPCODE_LANE_LINK);
        clearNode(ODR_OPCODE_LANE_WIDTH);
        clearNode(
            ODR_OPCODE_LANE_MATERIAL);
        clearNode(ODR_OPCODE_LANE_VISIBILITY);
        break;
      case ODR_OPCODE_SIGNAL:
        clearNode(ODR_OPCODE_LANE_VALIDITY);
        clearNode(
            ODR_OPCODE_SIGNAL_DEPEND);
        break;
      case ODR_OPCODE_OBJECT:
        clearNode(
            ODR_OPCODE_LANE_VALIDITY);
        break;
      case ODR_OPCODE_CONTROLLER:
        clearNode(
            ODR_OPCODE_CONTROL_ENTRY);
        break;
      case ODR_OPCODE_JUNCTION_HEADER:
        clearNode(
            ODR_OPCODE_JUNCTION_LINK);
        clearNode(ODR_OPCODE_JUNCTION_PRIORITY);
        clearNode(
            ODR_OPCODE_JUNCTION_CONTROL);
        break;
      case ODR_OPCODE_JUNCTION_LINK:
        clearNode(
            ODR_OPCODE_JUNCTION_LANE_LINK);
        break;
      }
    }
    return true;
  }
  Node *HierCoord::findNode(
      const unsigned int type)
  {
    checkSize(type);
    return mNodeAccessVec[type];
  }
  bool
  HierCoord::registerNode(Node *node, const unsigned int type)
  {
    if (mLocked)
      return false;
    if (node)
      return keepNode(node);
    return clearNode(type);
  }
  void HierCoord::lock() { mLocked = true; }
  void HierCoord::unlock() { mLocked = false; }
  void HierCoord::backup()
  {
    mNodeAccessVecBackup.clear();
    mNodeAccessVecBackup.resize(mNodeAccessVec.size(), 0);
    for (unsigned int i = 0; i < mNodeAccessVec.size(); i++)
      mNodeAccessVecBackup[i] =
          mNodeAccessVec[i];
  }
  void HierCoord::restore()
  {
    mNodeAccessVec.clear();
    mNodeAccessVec.resize(mNodeAccessVecBackup.size(), 0);
    for (unsigned int i = 0; i <
                             mNodeAccessVec.size();
         i++)
      mNodeAccessVec[i] = mNodeAccessVecBackup[i];
  }
  void
  HierCoord::checkSize(const unsigned int type)
  {
    if (type < mNodeAccessVec.size())
      return;
    mNodeAccessVec.resize(type + 1, 0);
  }
} // namespace OpenDrive
