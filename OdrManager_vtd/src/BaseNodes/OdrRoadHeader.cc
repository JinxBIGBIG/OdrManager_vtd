
#include "OdrGeoHeader.hh"
#include "OdrElevation.hh"
#include "OdrLateralShape.hh"
#include "OdrCrossfall.hh"
#include "OdrSuperelevation.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrRoadLink.hh"
#include "OdrSignal.hh"
#include "OdrRoadHeader.hh"
#include "OdrJuncLink.hh"
#include "OdrJuncHeader.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadType.hh"
#include "OdrLaneOffset.hh"
#include "OdrLaneLink.hh"
#include "OdrBbox.hh"
#include "OdrSurfaceCRG.hh"
#include "OdrRailroadSwitch.hh"
#include "OdrTunnel.hh"
#include "OdrBridge.hh"
#include "OdrObject.hh"
#include "OdrRepeat.hh"
#include "OdrRoadData.hh"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef LINUX
#define copysign(x, y) (y < 0 ? (-fabs(x)) : fabs(x))
#endif
namespace OpenDrive
{
  RoadHeader::RoadHeader() : Node("\x52\x6f\x61\x64\x48\x65\x61\x64\x65\x72"),
                             mContObjTesselation(2.0f),
                             mResolveRepeatedObjects(true), mPredElemDir(0), mSuccElemDir(0), 
                             mJuncPredElemDir(0), mJuncSuccElemDir(0), mPredS(0.0), mSuccS(0.0), 
                             mJuncPredS(0.0), mJuncSuccS(0.0), mPredecessor(0), mSuccessor(0), 
                             mJuncSuccessor(0), mJuncPredecessor(0), mJuncHeader(0),
                             mBbox(0), mHasSurfaceData(false)
  {
    mOpcode = ODR_OPCODE_ROAD_HEADER;
    mMaxHalfWidth = 0.0;
  }

  RoadHeader::RoadHeader(RoadHeader *Nf2Ao) : 
    Node(Nf2Ao), mPredElemDir(0),mSuccElemDir(0), mJuncPredElemDir(0), 
    mJuncSuccElemDir(0), mPredS(0.0), mSuccS(0.0), mPredecessor(0), 
    mSuccessor(0), mJuncSuccessor(0), mJuncPredecessor(0), mJuncHeader(0)
  {
    mName = Nf2Ao->mName;
    mLength = Nf2Ao->mLength;
    mId = Nf2Ao->mId;
    mIdAsString = Nf2Ao->mIdAsString;
    mJuncNo = Nf2Ao->mJuncNo;
    mJuncNoAsString = Nf2Ao->mJuncNoAsString;
    mRHT = Nf2Ao->mRHT;
    mMaxHalfWidth = Nf2Ao->mMaxHalfWidth;
    mContObjTesselation = Nf2Ao->mContObjTesselation;
    mResolveRepeatedObjects = Nf2Ao->mResolveRepeatedObjects;
    if ( Nf2Ao->getBoundingBox())
      mBbox = new Bbox(Nf2Ao->getBoundingBox());
  }

  RoadHeader::~RoadHeader()
  {
    if (mBbox)
      delete mBbox;
  }

  double RoadHeader::getLength() const {
    return mLength;
  }

  unsigned int RoadHeader::getId() const {
    return mId;
  }

  std::string RoadHeader::getIdStr() const {
    return mIdAsString;
  }

  int RoadHeader::getJunctionNo() const {
    return mJuncNo;
  }

  void RoadHeader::printData() const
  {
    fprintf(
        stderr, "\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73"
                "\n",
        mName
            .c_str());
    fprintf(stderr,
            "\x4c\x65\x6e\x67\x74\x68\x3a\x20\x20\x20\x20\x20\x20\x25\x2e\x34\x66"
            "\n",
            mLength);
    fprintf(stderr,
            "\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x28\x25\x64\x29"
            "\n",
            mIdAsString.c_str(), mId);
    fprintf(stderr,
            "\x4a\x75\x6e\x63\x2e\x20\x4e\x6f\x2e\x3a\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64"
            "\n",
            mJuncNoAsString.c_str(), mJuncNo);
    fprintf(stderr,
            "\x48\x61\x6c\x66\x20\x57\x69\x64\x74\x68\x2e\x3a\x20\x25\x2e\x34\x66"
            "\n",
            mMaxHalfWidth);
    fprintf(stderr,
            "\x72\x75\x6c\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e"
            "\n",
            mRHT
                ? "\x52\x48\x54"
                : "\x4c\x48\x54");
  }

  bool RoadHeader::read(ReaderXML *F3vnM)
  {
    mName = F3vnM->getString("\x6e\x61\x6d\x65");
    mLength = F3vnM->getDouble("\x6c\x65\x6e\x67\x74\x68");
    mIdAsString = F3vnM->getString("\x69\x64");
    mId = F3vnM->getUInt("\x69\x64");
    mJuncNo = F3vnM->getInt("\x6a\x75\x6e\x63\x74\x69\x6f\x6e");
    mJuncNoAsString = F3vnM->getString("\x6a\x75\x6e\x63\x74\x69\x6f\x6e");
    mRHT = !(F3vnM->getString("\x72\x75\x6c\x65") == std::string("\x4c\x48\x54"));
    if (!mJuncNoAsString.length())
      mJuncNo = -1;
    return true;
  }

  GeoHeader *RoadHeader::getFirstGeoHeader() { 
    return reinterpret_cast<GeoHeader *>(getChild(ODR_OPCODE_GEO_HEADER)); 
  }

  GeoHeader *RoadHeader::getLastGeoHeader()
  {
    GeoHeader *lXc5v = reinterpret_cast<GeoHeader *>(getChild(ODR_OPCODE_GEO_HEADER));
    while (lXc5v && lXc5v->getRight())
      lXc5v = reinterpret_cast<GeoHeader *>(lXc5v->getRight());
    return lXc5v;
  }

  Elevation *RoadHeader::getFirstElevation() { 
    return reinterpret_cast<Elevation *>(getChild(ODR_OPCODE_ELEVATION)); 
  }
      
  LateralShape *RoadHeader::getFirstLateralShape() { 
    return reinterpret_cast<LateralShape *>(getChild(ODR_OPCODE_LATERAL_SHAPE));
  }

  Crossfall *RoadHeader::getFirstCrossfall() { 
    return reinterpret_cast<Crossfall *>(getChild(ODR_OPCODE_CROSSFALL)); 
  }

  Superelevation *RoadHeader::getFirstSuperelevation() { 
    return reinterpret_cast<Superelevation *>(getChild(ODR_OPCODE_SUPERELEVATION)); 
  }

  LaneSection *RoadHeader::getFirstLaneSection() { 
    return reinterpret_cast<LaneSection *>(getChild(ODR_OPCODE_LANE_SECTION)); 
  }

  LaneSection *RoadHeader::getLastLaneSection() { 
    return reinterpret_cast<LaneSection *>(getLastChild(ODR_OPCODE_LANE_SECTION));
  }

  LaneOffset *RoadHeader::getFirstLaneOffset() {
    return reinterpret_cast<LaneOffset *>(getChild(ODR_OPCODE_LANE_OFFSET));
  }

  LaneOffset *RoadHeader::getLastLaneOffset() {
    return reinterpret_cast<LaneOffset *>(getLastChild(ODR_OPCODE_LANE_OFFSET));
  }
  
  Signal *RoadHeader::getFirstSignal() { 
    return reinterpret_cast<Signal *>(getChild(ODR_OPCODE_SIGNAL)); 
  }

  Tunnel *RoadHeader::getFirstTunnel() { 
    return reinterpret_cast<Tunnel *>(getChild(ODR_OPCODE_TUNNEL)); 
  }

  Bridge *RoadHeader::getFirstBridge()
  {
    return reinterpret_cast<Bridge *>(getChild(ODR_OPCODE_BRIDGE));
  }

  Object *RoadHeader ::getFirstObject() { 
    return reinterpret_cast<Object *>(getChild(ODR_OPCODE_OBJECT)); 
  }

  RoadType *RoadHeader::getFirstRoadType() { 
    return reinterpret_cast<RoadType *>(getChild(ODR_OPCODE_ROAD_TYPE)); 
  }

  RoadLink *RoadHeader::getFirstLink() { 
    return reinterpret_cast<RoadLink *>(getChild(ODR_OPCODE_ROAD_LINK)); 
  }

  RoadLink *RoadHeader ::getPredecessorLink(unsigned short type)
  {
    RoadLink *me3OO = getFirstLink();
    while (me3OO)
    {
      if (me3OO->mType == ODR_LINK_TYPE_PREDECESSOR && (me3OO->mElemType == type || type == ODR_TYPE_NONE))
      {
        mPredElemDir = me3OO->mElemDir;
        return me3OO;
      }
      me3OO = reinterpret_cast<RoadLink *>(me3OO->getRight());
    }
    return 0;
  }

  RoadLink *RoadHeader::getSuccessorLink(unsigned short type)
  {
    RoadLink *link = getFirstLink();
    while (link)
    {
      if (link->mType == ODR_LINK_TYPE_SUCCESSOR && (link->mElemType == type || type == ODR_TYPE_NONE))
      {
        mSuccElemDir = link->mElemDir;
        return link;
      }
      link = reinterpret_cast<RoadLink *>(link->getRight());
    }
    return 0;
  }

  RoadHeader *RoadHeader ::getPredecessor(bool uQCJs)
  {
    Node *node = getPredecessorNode();
    if (!node)
      return 0;
    if (node->getOpcode() == ODR_OPCODE_ROAD_HEADER)
      return reinterpret_cast<RoadHeader *>(node);
    if (node->getOpcode() == ODR_OPCODE_RAILROAD_SWITCH)
      return reinterpret_cast<RoadHeader *>(node->getParent());
    if (uQCJs && (node->getOpcode() == ODR_OPCODE_JUNCTION_HEADER))
      return mJuncPredecessor;
    return 0;
  }

  RoadHeader *RoadHeader::getSuccessor(bool uQCJs)
  {
    Node *node = getSuccessorNode();
    if (!node)
      return 0;
    if (node->getOpcode() == ODR_OPCODE_ROAD_HEADER)
      return reinterpret_cast<RoadHeader *>(node);
    if (node->getOpcode() == ODR_OPCODE_RAILROAD_SWITCH)
      return reinterpret_cast<RoadHeader *>(node->getParent());
    if (uQCJs && (node->getOpcode() == ODR_OPCODE_JUNCTION_HEADER))
      return mJuncSuccessor;
    return 0;
  }

  Node *RoadHeader::getPredecessorNode() { 
    return mPredecessor; 
  }

  Node *RoadHeader::getSuccessorNode()
  {
    return mSuccessor;
  }

  const unsigned char &RoadHeader::getPredecessorDir() const
  {
    if (!mPredecessor)
      return mJuncPredElemDir;
    if ((mPredecessor->getOpcode() ==ODR_OPCODE_ROAD_HEADER) ||
        (mPredecessor->getOpcode() == ODR_OPCODE_RAILROAD_SWITCH))
      return mPredElemDir;
    return mJuncPredElemDir;
  }

  const unsigned char &RoadHeader::getSuccessorDir() const
  {
    if (!mSuccessor)
      return mJuncSuccElemDir;
    if ((mSuccessor->getOpcode() == ODR_OPCODE_ROAD_HEADER) || (mSuccessor->getOpcode() ==
                                                                ODR_OPCODE_RAILROAD_SWITCH))
      return mSuccElemDir;
    return mJuncSuccElemDir;
  }

  double RoadHeader::getPredecessorS() const
  {
    if (!mPredecessor)
      return 0.0;
    if ((mPredecessor->getOpcode() == ODR_OPCODE_ROAD_HEADER) ||
        (mPredecessor->getOpcode() == ODR_OPCODE_RAILROAD_SWITCH))
      return mPredS;
    return mJuncPredS;
  }

  double RoadHeader::getSuccessorS() const
  {
    if (!mSuccessor)
      return 0.0;
    if ((mSuccessor->getOpcode() ==
         ODR_OPCODE_ROAD_HEADER) ||
        (mSuccessor->getOpcode() == ODR_OPCODE_RAILROAD_SWITCH))
      return mSuccS;
    return mJuncSuccS;
  }

  void RoadHeader::calcRRLinks()
  {
    static bool Y3vvh = false;
    if (Y3vvh)
    {
      if (mPredecessor || mSuccessor)
        fprintf(stderr,
                "\x52\x6f\x61\x64\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x52\x52\x4c\x69\x6e\x6b\x73\x3a\x20\x72\x6f\x61\x64\x20\x3c\x25\x73\x3e\x20\x25\x64\x20\x68\x61\x73\x20\x73\x77\x69\x74\x63\x68\x20\x61\x73\x20\x6e\x65\x69\x67\x68\x62\x6f\x72\x73\x3a\x20\x70\x72\x65\x64\x65\x63\x65\x73\x73\x6f\x72\x20\x3d\x20\x25\x70\x2c\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x20\x3d\x20\x25\x70\x20"
                "\n",
                mIdAsString.c_str(), mId, mPredecessor, mSuccessor);
      if (mPredecessor)
        mPredecessor->print(true, false);
      if (mSuccessor)
        mSuccessor->print(true, false);
    }
  }

  void RoadHeader ::calcPredecessor()
  {
    mPredecessor = getNeighborFromLink(getPredecessorLink(ODR_TYPE_NONE));
    if (!mPredecessor)
      return;
    if (mPredecessor->getOpcode() == ODR_OPCODE_ROAD_HEADER)
    {
      RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mPredecessor);
      mPredS = (mPredElemDir == ODR_LINK_POINT_START) ? 0.0 : M8Gux->mLength;
    }
  }

  void RoadHeader::calcSuccessor()
  {
    mSuccessor = getNeighborFromLink(getSuccessorLink(ODR_TYPE_NONE));
    if (!mSuccessor)
      return;
    if (mSuccessor->getOpcode() == ODR_OPCODE_ROAD_HEADER)
    {
      RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mSuccessor);
      mSuccS = (mSuccElemDir == ODR_LINK_POINT_START) ? 0.0 : M8Gux->mLength;
    }
  }

  void RoadHeader::calcPrepareData()
  {
    if (!RoadData::getInstance())
    {
      fprintf(stderr,
              "\x52\x6f\x61\x64\x48\x65\x61\x64\x65\x72\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21"
              "\n"
              "\x20");
      return;
    }
    mPredecessor = 0;
    mSuccessor = 0;
    if (!mPredecessor)
      calcPredecessor();
    if (!mSuccessor)
      calcSuccessor();
    sortChildren();
    mHasSurfaceData = getChild(ODR_OPCODE_SURFACE_CRG) != 0;
    if (mResolveRepeatedObjects)
    {
      Object *Hh1VH = reinterpret_cast<Object *>(getChild(ODR_OPCODE_OBJECT));
      while (Hh1VH)
      {
        Repeat *LL1iw = Hh1VH->getFirstRepeat();
        Object *HpqGJ = Hh1VH;
        bool tGL1B = true;
        while (LL1iw)
        {
          double hAzaz = LL1iw->mDistance;
          int qf6WO = 0;
          bool Dpx_l = false;
          if (LL1iw->mLength > 0.0)
          {
            if ((hAzaz == 0.0) && (mContObjTesselation > 0.0))
            {
              Hh1VH->mSizeX = mContObjTesselation;
              hAzaz = mContObjTesselation;
              Dpx_l = true;
            }
            if (hAzaz > 0.0)
              qf6WO = 1 + (LL1iw->mLength / hAzaz);
          }
          int gVCjm = tGL1B ? 1 : 0;
          if (qf6WO > gVCjm)
          {
            for (int ggPMx = gVCjm; ggPMx < qf6WO; ggPMx++)
            {
              double ERUXs = (1.0 * ggPMx) / (1.0 * (qf6WO - 1));
              Object *TmDuj = reinterpret_cast<Object *>(Hh1VH->getCopy(true));
              if (TmDuj)
              {
                TmDuj->mHasRepeat = false;
                TmDuj->mS = LL1iw->mS + ggPMx *
                                            hAzaz;
                TmDuj->mT = LL1iw->mTStart + ERUXs * (LL1iw->mTEnd - LL1iw->mTStart);
                TmDuj->mZ = LL1iw->mZOffsetStart + ERUXs * (LL1iw->mZOffsetEnd - LL1iw->mZOffsetStart);
                TmDuj->mSizeY = LL1iw->mWidthStart + ERUXs * (LL1iw->mWidthEnd - LL1iw->mWidthStart);
                TmDuj->mSizeZ = LL1iw->mHeightStart + ERUXs * (LL1iw->mHeightEnd - LL1iw->mHeightStart);
                if (Dpx_l)
                {
                  TmDuj->mLength = mContObjTesselation;
                  if ((TmDuj->mLength + TmDuj->mS) > mLength)
                    TmDuj->mLength = mLength - TmDuj->mS;
                }
                TmDuj->calcPrepareData();
                Node *PxOhF = 0;
                while ((
                    PxOhF = TmDuj->getChild(ODR_OPCODE_REPEAT)))
                  delete PxOhF;
                HpqGJ->addRightSibling(
                    TmDuj);
                HpqGJ = TmDuj;
              }
            }
          }
          if (!LL1iw->getLeft())
          {
            Hh1VH->mS = LL1iw->mS;
            Hh1VH->mT = LL1iw
                            ->mTStart;
            Hh1VH->mZ = LL1iw->mZOffsetStart;
            Hh1VH->mSizeY = LL1iw->mWidthStart;
            Hh1VH
                ->mSizeZ = LL1iw->mHeightStart;
          }
          if (Dpx_l)
          {
            Hh1VH->mLength = mContObjTesselation;
            if ((
                    Hh1VH->mLength + Hh1VH->mS) > mLength)
              Hh1VH->mLength = mLength - Hh1VH->mS;
          }
          LL1iw =
              reinterpret_cast<Repeat *>(LL1iw->getRight());
          tGL1B = false;
        }
        Hh1VH->mHasRepeat =
            false;
        delete LL1iw;
        Hh1VH = reinterpret_cast<Object *>(Hh1VH->getRight());
      }
    }
    if (!getRight())
    {
      RoadHeader *c5SkH = this;
      while (c5SkH)
      {
        if (c5SkH->getPredecessor())
        {
          if (
              c5SkH->getPredecessor()->getOpcode() == ODR_OPCODE_ROAD_HEADER)
            c5SkH->getPredecessor()->registerNeighbor(c5SkH);
        }
        if (c5SkH->getSuccessor())
        {
          if (c5SkH->getSuccessor()->getOpcode() == ODR_OPCODE_ROAD_HEADER)
            c5SkH->getSuccessor()->registerNeighbor(c5SkH);
        }
        RailroadSwitch *VRE77 = reinterpret_cast<RailroadSwitch *>(
            c5SkH->getChild(ODR_OPCODE_RAILROAD_SWITCH));
        while (VRE77)
        {
          RoadHeader *dL3OP =
              reinterpret_cast<RoadHeader *>(RoadData::getInstance()->getNodeFromId(
                  ODR_OPCODE_ROAD_HEADER, VRE77->mSideTrackIdAsString));
          if (dL3OP)
          {
            if (fabs(VRE77->mSideTrackS) < 1.0e-2)
            {
              dL3OP->mPredecessor = VRE77;
              dL3OP->mPredElemDir = (VRE77->mDir == ODR_DIRECTION_PLUS) ? ODR_LINK_POINT_END : ODR_LINK_POINT_START;
              dL3OP->mPredS =
                  VRE77->mS;
            }
            else
            {
              dL3OP->mSuccessor = VRE77;
              dL3OP->mSuccElemDir = (VRE77->mDir ==
                                     ODR_DIRECTION_PLUS)
                                        ? ODR_LINK_POINT_END
                                        : ODR_LINK_POINT_START;
              dL3OP->mSuccS = VRE77
                                  ->mS;
            }
          }
          VRE77 = reinterpret_cast<RailroadSwitch *>(VRE77->getRight());
        }
        c5SkH =
            reinterpret_cast<RoadHeader *>(c5SkH->getLeft());
      }
    }
  }

  void RoadHeader::calcPostPrepareData()
  {
    calcBoundingBox();
    mJuncPredecessor = 0;
    mJuncSuccessor = 0;
    for (
        int i = 0; i < 2; i++)
    {
      Node *oMGBk = i ? mSuccessor : mPredecessor;
      RoadHeader *NTG3j = 0;
      unsigned char B9x6g = 0;
      if (oMGBk && (oMGBk->getOpcode() == ODR_OPCODE_JUNCTION_HEADER))
      {
        JuncHeader *GqNs7 = reinterpret_cast<JuncHeader *>(oMGBk);
        JuncLink *jMwpP = GqNs7->getFirstLink();
        double AIdPo = 0.0;
        while (jMwpP)
        {
          if ((jMwpP->mIncomingRoad == this) &&
              jMwpP->mConnectingRoad)
          {
            if (!NTG3j || (fabs(jMwpP->mTurnAngle) < fabs(AIdPo)))
            {
              NTG3j =
                  jMwpP->mConnectingRoad;
              AIdPo = jMwpP->mTurnAngle;
              B9x6g = jMwpP->mDir;
            }
          }
          jMwpP =
              reinterpret_cast<JuncLink *>(jMwpP->getRight());
        }
        if (i)
        {
          mJuncSuccessor = NTG3j;
          mJuncSuccElemDir = B9x6g;
          if (NTG3j)
          {
            mJuncSuccS = (B9x6g == ODR_LINK_POINT_START) ? 0.0 : NTG3j->mLength;
          }
        }
        else
        {
          mJuncPredecessor = NTG3j;
          mJuncPredElemDir = B9x6g;
          if (NTG3j)
          {
            mJuncPredS = (B9x6g == ODR_LINK_POINT_START) ? 0.0 : NTG3j->mLength;
          }
        }
      }
    }
    calcRRLinks();
  }

  Node *RoadHeader::getCopy(bool Mupxf)
  {
    Node *dzamm = new RoadHeader(this);
    if (Mupxf)
      deepCopy(dzamm);
    calcPrepareData();
    return dzamm;
  }

  void RoadHeader::invalidateLanes(int P4f3f, bool Axzq_)
  {
    LaneSection *H4prs = reinterpret_cast<LaneSection *>(getChild(ODR_OPCODE_LANE_SECTION));
    while (H4prs)
    {
      int DHeCf = P4f3f;
      Lane *lane =
          reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
      while (lane)
      {
        if ((lane->mId != P4f3f) && (!Axzq_ || (Axzq_ && lane->isDriveable())))
          lane->mType = 0;
        if (lane->mId ==
            P4f3f)
        {
          LaneLink *hYfdO = lane->getSuccessorLink();
          if (hYfdO)
            DHeCf = hYfdO->mLaneId;
        }
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
      if (H4prs->isCloned())
      {
        Lane *lane =
            reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
        while (lane)
        {
          if ((lane->mId != 0) && ((copysign(1.0, lane->mId) != copysign(1.0, P4f3f)) || (abs(lane->mId) > abs(
                                                                                                               P4f3f))))
          {
            lane->isolate();
            delete lane;
            lane = reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
          }
          else
            lane = reinterpret_cast<Lane *>(lane->getRight());
        }
      }
      P4f3f = DHeCf;
      H4prs = reinterpret_cast<LaneSection *>(H4prs->getRight());
    }
  }

  Lane *RoadHeader::getLaneInFirstSection(int f4e6v)
  {
    LaneSection *H4prs = reinterpret_cast<
        LaneSection *>(getChild(ODR_OPCODE_LANE_SECTION));
    if (!H4prs)
      return 0;
    Lane *lane =
        reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
    while (lane)
    {
      if (lane->mId == f4e6v)
        return lane;
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    return 0;
  }

  Node *RoadHeader::getJunction() { 
    return mJuncHeader; 
  }

  void RoadHeader::setJunction(Node *GqNs7) { 
    mJuncHeader = GqNs7; 
  }

  unsigned int RoadHeader::getNoDriveableLanes()
  {
    unsigned int ZwISv = 0;
    LaneSection *H4prs = reinterpret_cast<LaneSection *>(getChild(
        ODR_OPCODE_LANE_SECTION));
    while (H4prs)
    {
      unsigned int f4e6v = 0;
      Lane *lane =
          reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
      while (lane)
      {
        if (lane->isDriveable())
          f4e6v++;
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
      if (f4e6v >
          ZwISv)
        ZwISv = f4e6v;
      H4prs = reinterpret_cast<LaneSection *>(H4prs->getRight());
    }
    return ZwISv;
  }

  Bbox *RoadHeader::getBoundingBox() { 
    return mBbox; 
  }

  bool RoadHeader::inBoundingBox(const double &x, const double &y)
  {
    if (!mBbox)
      return true;
    return mBbox->containsXY(x, y);
  }

  RoadHeader *RoadHeader::getNeighbor(unsigned int hFNbl)
  {
    if (
        hFNbl >= mNeighbors.size())
      return 0;
    return mNeighbors.at(hFNbl);
  }

  void RoadHeader::
      registerNeighbor(RoadHeader *XrghW)
  {
    if (!XrghW)
      return;
    if ((XrghW == this) || (XrghW == mPredecessor) || (XrghW == mSuccessor))
      return;
    mNeighbors.push_back(XrghW);
  }

  Node *RoadHeader::getNeighborFromLink(RoadLink *hYfdO)
  {
    if (!hYfdO)
      return 0;
    if (hYfdO->mElemType == ODR_TYPE_ROAD)
    {
      if (!RoadData::getInstance())
      {
        fprintf(stderr,
                "\x52\x6f\x61\x64\x48\x65\x61\x64\x65\x72\x3a\x3a\x67\x65\x74\x4e\x65\x69\x67\x68\x62\x6f\x72\x46\x72\x6f\x6d\x4c\x69\x6e\x6b\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21"
                "\n"
                "\x20");
        return 0;
      }
      RoadHeader *REKut = reinterpret_cast<RoadHeader *>(RoadData::getInstance()->getNodeFromId(ODR_OPCODE_ROAD_HEADER, hYfdO->mElemIdAsString));
      return REKut;
    }
    if (hYfdO->mElemType == ODR_TYPE_JUNCTION)
    {
      JuncHeader *GqNs7 = reinterpret_cast<
          JuncHeader *>(getParent()->getChild(ODR_OPCODE_JUNCTION_HEADER));
      while (GqNs7 && (GqNs7->mIdAsString != hYfdO->mElemIdAsString))
        GqNs7 = reinterpret_cast<JuncHeader *>(
            GqNs7->getRight());
      return GqNs7;
    }
    return 0;
  }

  void RoadHeader::sortChildren()
  {
    bool Vc1df = false;
    do
    {
      Vc1df = false;
      Node *Ubpbz = getChild(ODR_OPCODE_SIGNAL);
      Node *nextNode;
      while (Ubpbz)
      {
        nextNode = Ubpbz->getRight();
        if (nextNode && nextNode->getS() < Ubpbz->getS())
        {
          Ubpbz->shiftRight();
          Vc1df = true;
        }
        else
          Ubpbz = nextNode;
      }
    } while (Vc1df);
    do
    {
      Vc1df = false;
      Node *Ubpbz = getChild(ODR_OPCODE_SIGNAL_REFERENCE);
      Node *nextNode;
      while (Ubpbz)
      {
        nextNode = Ubpbz->getRight();
        if (nextNode && nextNode->getS() < Ubpbz->getS())
        {
          Ubpbz->shiftRight();
          Vc1df = true;
        }
        else
          Ubpbz = nextNode;
      }
    } while (Vc1df);
    do
    {
      Vc1df =
          false;
      Node *Ubpbz = getChild(ODR_OPCODE_LATERAL_SHAPE);
      Node *nextNode;
      while (Ubpbz)
      {
        nextNode = Ubpbz->getRight();
        if (nextNode && (nextNode->getS() < Ubpbz->getS()))
        {
          Ubpbz
              ->shiftRight();
          Vc1df = true;
        }
        else
          Ubpbz = nextNode;
      }
    } while (Vc1df);
    LateralShape *Ubpbz = reinterpret_cast<LateralShape *>(getLastChild(ODR_OPCODE_LATERAL_SHAPE));
    double
        sEnd = mLength;
    while (Ubpbz)
    {
      Ubpbz->mSEnd = sEnd;
      LateralShape *SICJS = reinterpret_cast<
          LateralShape *>(Ubpbz->getLeft());
      while (SICJS)
      {
        if (SICJS->mS == Ubpbz->mS)
          SICJS->mSEnd = Ubpbz->mSEnd;
        else
        {
          sEnd = Ubpbz->mS;
          break;
        }
        SICJS = reinterpret_cast<
            LateralShape *>(SICJS->getLeft());
      }
      Ubpbz = SICJS;
    }
    do
    {
      Vc1df = false;
      LateralShape *Ubpbz = reinterpret_cast<LateralShape *>(getChild(ODR_OPCODE_LATERAL_SHAPE));
      LateralShape *nextNode = 0;
      while (Ubpbz)
      {
        nextNode = reinterpret_cast<LateralShape *>(
            Ubpbz->getRight());
        if (nextNode)
        {
          if (nextNode->mS == Ubpbz->mS)
          {
            if (nextNode->mT <
                Ubpbz->mT)
            {
              Ubpbz->shiftRight();
              Vc1df = true;
            }
            else
              Ubpbz = nextNode;
          }
          else if (Vc1df)
            break;
          else
            Ubpbz = nextNode;
        }
        else
          Ubpbz = nextNode;
      }
    } while (Vc1df);
    Ubpbz =
        reinterpret_cast<LateralShape *>(getChild(ODR_OPCODE_LATERAL_SHAPE));
    while (Ubpbz)
    {
      LateralShape *nextNode = reinterpret_cast<LateralShape *>(Ubpbz->getRight());
      if (
          nextNode)
      {
        if (nextNode->mS == Ubpbz->mS)
          Ubpbz->mTEnd = nextNode->mT;
        else
          Ubpbz->mTEnd = 999999.0;
      }
      else
        Ubpbz->mTEnd = 999999.0;
      Ubpbz = nextNode;
    }
  }

  void RoadHeader::calcBoundingBox(const double &gu5bP)
  {
    if (!mBbox)
      mBbox = new Bbox();
    double Fc2XK =
        gu5bP;
    double rxv4_ = 0.0;
    if (Fc2XK == 0.0)
    {
      LaneSection *icbqk = getFirstLaneSection();
      while (icbqk)
      {
        double width = icbqk->getMaxHalfWidth();
        Fc2XK = width > Fc2XK ? width : Fc2XK;
        icbqk = reinterpret_cast<LaneSection *>(icbqk->getRight());
      }
      LaneOffset *hYLzo =
          reinterpret_cast<LaneOffset *>(getChild(ODR_OPCODE_LANE_OFFSET));
      while (hYLzo)
      {
        double offset = hYLzo->getMaxValue();
        rxv4_ = offset > rxv4_ ? offset : rxv4_;
        hYLzo =
            reinterpret_cast<LaneOffset *>(hYLzo->getRight());
      }
      Fc2XK += rxv4_;
    }
    if (Fc2XK == 0.0)
      Fc2XK = 30.0;
    mMaxHalfWidth = Fc2XK;
    GeoHeader *xjOaT = getFirstGeoHeader();
    while (xjOaT)
    {
      xjOaT->calcBoundingBox(Fc2XK);
      mBbox->add(xjOaT->getBoundingBox());
      xjOaT =
          reinterpret_cast<GeoHeader *>(xjOaT->getRight());
    }
  }

  bool RoadHeader::hasSurfaceData(const unsigned short &TCz3u)
  {
    if (!mHasSurfaceData)
      return false;
    if (
        TCz3u == SurfaceCRG::sPurposeAny)
      return true;
    SurfaceCRG *o055c = reinterpret_cast<
        SurfaceCRG *>(getChild(ODR_OPCODE_SURFACE_CRG));
    while (o055c)
    {
      if (o055c->mPurpose &
          TCz3u)
        return true;
      o055c = reinterpret_cast<SurfaceCRG *>(o055c->getRight());
    }
    return false;
  }

  SurfaceCRG *RoadHeader::s2surface(const double &BJBDA, const unsigned short
                                                             &TCz3u)
  {
    if (!hasSurfaceData(TCz3u))
      return 0;
    SurfaceCRG *oV99J = reinterpret_cast<
        SurfaceCRG *>(getChild(ODR_OPCODE_SURFACE_CRG));
    SurfaceCRG *hjyW8 = 0;
    while (oV99J)
    {
      if ((oV99J->mPurpose & TCz3u) && (oV99J->mS <= BJBDA) && (oV99J->mSEnd >= BJBDA))
        hjyW8 =
            oV99J;
      oV99J = reinterpret_cast<SurfaceCRG *>(oV99J->getRight());
    }
    return hjyW8;
  }

  double RoadHeader::getMaxCurvatureInRange(const double &start, const double &_aUXU)
  {
    double Yrh_j = (start < _aUXU) ? start : _aUXU;
    double LI3i5 = (start < _aUXU) ? _aUXU : start;
    double d4xeY = 0.0;
    GeoHeader *xjOaT = reinterpret_cast<GeoHeader *>(getChild(
        ODR_OPCODE_GEO_HEADER));
    while (xjOaT)
    {
      if (xjOaT->mS > LI3i5)
        return d4xeY;
      else if ((
                   xjOaT->mS + xjOaT->mLength) > Yrh_j)
      {
        double L9KFn = xjOaT->getMaxCurvatureInRange(
            Yrh_j, LI3i5);
        d4xeY = (fabs(d4xeY) < fabs(L9KFn)) ? L9KFn : d4xeY;
      }
      xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getRight());
    }
    return d4xeY;
  }

  void RoadHeader::setId(const unsigned int &id, bool vSWEf)
  {
    mId = id;
    if (!vSWEf)
      return;
    char iB6gF[64];
    sprintf(iB6gF, "\x25\x64", mId);
    mIdAsString = std::string(iB6gF);
  }

  void RoadHeader::setJuncNo(
      const int &id, bool vSWEf)
  {
    mJuncNo = id;
    if (!vSWEf)
      return;
    char iB6gF[64];
    sprintf(
        iB6gF, "\x25\x64", mJuncNo);
    mJuncNoAsString = std::string(iB6gF);
  }

  void RoadHeader::printNeighbors()
  {
    for (std::vector<RoadHeader *>::iterator VfDVC = mNeighbors.begin(); VfDVC != mNeighbors.end(); VfDVC++)
      fprintf(stderr,
              "\x25\x73\x2c\x20\x6e\x65\x69\x67\x68\x62\x6f\x72\x20\x69\x73\x20\x25\x73"
              "\n",
              mIdAsString.c_str(), (*VfDVC)->mIdAsString.c_str());
  }
} // namespace OpenDrive
