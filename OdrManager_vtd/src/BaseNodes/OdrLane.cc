
#include "OdrLane.hh"
#include "OdrLaneSection.hh"
#include "OdrLaneWidth.hh"
#include "OdrLaneLink.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadMark.hh"
#include "OdrRoadHeader.hh"
#include "OdrTrafficObject.hh"
#include "OdrLaneBorder.hh"
#include <stdio.h>
#include <iostream>
namespace OpenDrive
{
  Lane::Lane() : Node("\x4c\x61\x6e\x65"), mPredElemDir(0),
                 mSuccElemDir(0), mPredecessor(0), mSuccessor(0)
  {
    mOpcode = ODR_OPCODE_LANE;
    mLevel = 2;
  }
  Lane::Lane(Lane *Nf2Ao) : Node(Nf2Ao)
  {
    mId = Nf2Ao->mId;
    mType = Nf2Ao->mType;
    mSurfLevel =
        Nf2Ao->mSurfLevel;
  }
  Lane::~Lane() {}
  void Lane::printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x49\x44\x3a\x20\x20\x20\x20\x25\x64"
            "\n",
            mId);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x25\x64"
            "\n",
            mType);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x4c\x65\x76\x65\x6c\x3a\x20\x25\x64"
            "\n",
            mSurfLevel);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x70\x72\x65\x64\x65\x63\x65\x73\x73\x6f\x72\x3a\x20\x30\x78\x25\x6c\x78"
            "\n",
            (unsigned long long)(mPredecessor));
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x3a\x20\x20\x20\x30\x78\x25\x6c\x78"
            "\n",
            (unsigned long long)(mSuccessor));
  }

  int Lane::getId() const {
    return mId;
  }

  unsigned int Lane::getType() const {
    return mType;
  }

  unsigned int Lane::getSurfLevel() const {
    return mSurfLevel;
  }

  bool Lane::read(ReaderXML *F3vnM)
  {
    mId = F3vnM->getInt("\x69\x64");
    mType = F3vnM->getOpcodeFromLaneType(F3vnM->getString(
        "\x74\x79\x70\x65"));
    mSurfLevel = (F3vnM->getString("\x6c\x65\x76\x65\x6c") ==
                  "\x74\x72\x75\x65") ||
                 (F3vnM->getString("\x6c\x65\x76\x65\x6c") == "\x31");
    return true;
  }
  LaneWidth *Lane::getFirstWidth() { 
    return reinterpret_cast<LaneWidth *>(getChild(ODR_OPCODE_LANE_WIDTH)); 
  }
  LaneBorder *Lane::getFirstBorder() { 
    return reinterpret_cast<LaneBorder *>(getChild(ODR_OPCODE_LANE_BORDER)); 
  }

  RoadMark *Lane::getFirstRoadMark() { 
    return reinterpret_cast<RoadMark *>(getChild(ODR_OPCODE_LANE_ROAD_MARK)); 
  }

  LaneLink *Lane::getFirstLink() { 
    return reinterpret_cast<LaneLink *>(getChild(ODR_OPCODE_LANE_LINK)); 
  }

  LaneLink *Lane::getPredecessorLink() { 
    return getLinkFromType(ODR_LINK_TYPE_PREDECESSOR); 
  }

  LaneLink* Lane::getSuccessorLink() { 
    return getLinkFromType(ODR_LINK_TYPE_SUCCESSOR); 
  }

  bool Lane::insertChildByS(Node *rkAEh)
  {
    if (!rkAEh)
      return false;
    Node *Gv01F = getChild(
        rkAEh->getOpcode());
    if (!Gv01F)
    {
      addChild(rkAEh);
      return true;
    }
    if (rkAEh->getOpcode() != ODR_OPCODE_TRAFFIC_OBJECT)
    {
      std::cerr << "\x4c\x61\x6e\x65\x3a\x3a\x69\x6e\x73\x65\x72\x74\x43\x68\x69\x6c\x64\x42\x79\x53\x3a\x20\x63\x61\x6e\x6e\x6f\x74\x20\x68\x61\x6e\x64\x6c\x65\x20\x6f\x74\x68\x65\x72\x20\x6f\x62\x6a\x65\x63\x74\x73\x20\x74\x68\x61\x6e\x20\x54\x72\x61\x66\x66\x69\x63\x4f\x62\x6a\x65\x63\x74\x73"
                << std::endl;
      return false;
    }
    TrafficObject *Tcyqo = reinterpret_cast<TrafficObject *>(
        rkAEh);
    TrafficObject *ONjYL = reinterpret_cast<TrafficObject *>(Gv01F);
    while (ONjYL)
    {
      TrafficObject *FssoZ = reinterpret_cast<TrafficObject *>(ONjYL->getRight());
      if (ONjYL
              ->mS > Tcyqo->mS)
      {
        ONjYL->addLeftSibling(rkAEh);
        break;
      }
      if (!FssoZ || (FssoZ && FssoZ->mS >= Tcyqo->mS))
      {
        ONjYL->addRightSibling(rkAEh);
        break;
      }
      ONjYL = FssoZ;
    }
    return true;
  }

  Lane *Lane::getPredecessor()
  {
    if (!mPredecessor)
      mPredecessor = calcPredecessor();
    return mPredecessor;
  }

  Lane *Lane::getSuccessor()
  {
    if (!mSuccessor)
      mSuccessor =
          calcSuccessor();
    return mSuccessor;
  }

  Lane *Lane::calcPredecessor()
  {
    LaneLink *hYfdO =
        getPredecessorLink();
    if (!hYfdO)
    {
      return 0;
    }
    mPredElemDir = ODR_LINK_POINT_END;
    LaneSection *H4prs = reinterpret_cast<LaneSection *>(getParent()->getLeft());
    if (!H4prs)
    {
      RoadHeader *tYqoL = reinterpret_cast<RoadHeader *>(getParent()->getParent());
      RoadHeader *J0QRu = tYqoL->getPredecessor();
      if (!J0QRu)
      {
        return 0;
      }
      mPredElemDir = tYqoL
                         ->getPredecessorDir();
      if (mPredElemDir == ODR_LINK_POINT_START)
        H4prs = J0QRu->getFirstLaneSection();
      else
        H4prs = J0QRu->getLastLaneSection();
    }
    if (!H4prs)
      return 0;
    return H4prs->getLaneFromId(hYfdO->mLaneId);
  }

  Lane *Lane::calcSuccessor()
  {
    LaneLink *hYfdO = getSuccessorLink();
    if (!hYfdO)
      return 0;
    mSuccElemDir =
        ODR_LINK_POINT_START;
    LaneSection *H4prs = reinterpret_cast<LaneSection *>(getParent()->getRight());
    if (!H4prs)
    {
      RoadHeader *tYqoL = reinterpret_cast<RoadHeader *>(
          getParent()->getParent());
      RoadHeader *J0QRu = tYqoL->getSuccessor();
      if (!J0QRu)
        return 0;
      mSuccElemDir = tYqoL->getSuccessorDir();
      if (mSuccElemDir ==
          ODR_LINK_POINT_START)
        H4prs = J0QRu->getFirstLaneSection();
      else
        H4prs = J0QRu->getLastLaneSection();
    }
    if (!H4prs)
      return 0;
    return H4prs->getLaneFromId(hYfdO->mLaneId);
  }

  const unsigned char &Lane::getPredecessorDir() const { return mPredElemDir; }

  const unsigned char &Lane::getSuccessorDir() const { return mSuccElemDir; }

  Node * Lane::getCopy(bool Mupxf)
  {
    Node *dzamm = new Lane(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }

  bool Lane::isDriveable(bool ce0Qj, bool lAdDx)
  {
    if (!mId)
      return false;
    bool U6Txr = false;
    if (ce0Qj)
      U6Txr = (mType == ODR_LANE_TYPE_DRIVING) || 
              (mType == ODR_LANE_TYPE_MWY_ENTRY) || 
              (mType == ODR_LANE_TYPE_MWY_EXIT) || 
              (mType == ODR_LANE_TYPE_ENTRY) || 
              (mType == ODR_LANE_TYPE_EXIT) || 
              (mType == ODR_LANE_TYPE_ON_RAMP) || 
              (mType == ODR_LANE_TYPE_OFF_RAMP) || 
              (mType == ODR_LANE_TYPE_CONNECTING_RAMP) || 
              (mType == ODR_LANE_TYPE_DRIVING_ROADWORKS);

    if (!lAdDx || U6Txr)
      return U6Txr;
    return (mType == ODR_LANE_TYPE_RAIL) ||
           (mType == ODR_LANE_TYPE_TRAM);
  }
  void Lane::getWidthMinMax(double &hAogV, double &Fc2XK)
  {
    hAogV = 0.0;
    Fc2XK = 0.0;
    LaneWidth *QgFeQ = getFirstWidth();
    bool XcSL1 = true;
    while (QgFeQ)
    {
      if (XcSL1)
        QgFeQ->getMinMax(hAogV,
                         Fc2XK);
      else
      {
        double min;
        double max;
        QgFeQ->getMinMax(min, max);
        hAogV = min < hAogV ? min : hAogV;
        Fc2XK = max > Fc2XK ? max : Fc2XK;
      }
      XcSL1 = false;
      QgFeQ = reinterpret_cast<LaneWidth *>(
          QgFeQ->getRight());
    }
  }
  
  LaneLink *Lane::getLinkFromType(unsigned short type)
  {
    LaneLink *hYfdO = getFirstLink();
    while (hYfdO)
    {
      if (hYfdO->mType == type)
        return hYfdO;
      hYfdO = reinterpret_cast<LaneLink *>(hYfdO->getRight());
    }
    return 0;
  }
} // namespace OpenDrive
