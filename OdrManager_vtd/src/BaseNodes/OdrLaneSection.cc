
#include "OdrLane.hh"
#include "OdrLaneWidth.hh"
#include "OdrLaneBorder.hh"
#include "OdrLaneSection.hh"
#include "OdrRoadHeader.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <stdlib.h>
namespace OpenDrive
{
  LaneSection::LaneSection() : Node("\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e"),
                               mSideMask(sLaneSecSideFlagAll),
                               mMaxHalfWidth(0.0)
  {
    mOpcode = ODR_OPCODE_LANE_SECTION;
    mLevel = 1;
  }

  LaneSection::LaneSection(LaneSection *Nf2Ao) : Node(Nf2Ao)
  {
    mS = Nf2Ao->mS;
    mSEnd = Nf2Ao->mSEnd;
    mSideMask = Nf2Ao->mSideMask;
    mMaxHalfWidth = Nf2Ao->getMaxHalfWidth();
  }

  LaneSection::~LaneSection() {}

  void LaneSection::printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x73\x3a\x20\x25\x2e\x38\x6c\x66\x20\x2d\x20\x25\x2e\x38\x6c\x66"
            "\n",
            mS, mSEnd);
    fprintf(stderr,
            "\x20\x20\x20\x20\x73\x69\x6e\x67\x6c\x65\x53\x69\x64\x65\x20\x3d\x20\x25\x73"
            "\n",
            (mSideMask != sLaneSecSideFlagAll) ? "\x74\x72\x75\x65" : "\x66\x61\x6c\x73\x65");
  }
  bool LaneSection::read(ReaderXML *F3vnM)
  {
    mS = F3vnM->getDouble("\x73");
    mSEnd = 0.0;
    RoadHeader *hdr = reinterpret_cast<RoadHeader *>(getParent());
    if (hdr)
      mSEnd = hdr->mLength;
    if ((F3vnM->getString("\x73\x69\x6e\x67\x6c\x65\x53\x69\x64\x65") ==
         "\x74\x72\x75\x65") ||
        (F3vnM->getString(
             "\x73\x69\x6e\x67\x6c\x65\x53\x69\x64\x65") == "\x31"))
    {
      mSideMask =
          sLaneSecSideFlagNone;
    }
    else
    {
      LaneSection *Wdjgq = reinterpret_cast<LaneSection *>(
          getLeft());
      if (Wdjgq)
      {
        if (Wdjgq->mSideMask == mSideMask)
          Wdjgq->mSEnd = mS;
      }
    }
    return true;
  }

  Lane *LaneSection::getLaneFromId(int id)
  {
    Lane *lane = reinterpret_cast<Lane *>(getChild());
    if (lane && (id > lane->mId))
      return 0;
    while (lane)
    {
      if (lane->mId == id)
        return lane;
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    return 0;
  }

  void LaneSection ::addBorderLanes(const double &LUtj3, const unsigned int &xb0Qj, const double &bOPv4,
                                    const unsigned int &sjRyt)
  {
    sortLanes();
    bool hlwWg = true;
    bool xre3q = true;
    if (
        mSideMask != sLaneSecSideFlagAll)
    {
      hlwWg = false;
      xre3q = false;
      Lane *lane =
          reinterpret_cast<Lane *>(getChild());
      while (lane)
      {
        if (lane->mId > 0)
          hlwWg = true;
        else if (lane->mId < 0)
          xre3q = true;
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
    }
    Lane *
        lane = reinterpret_cast<Lane *>(getChild());
    int JQlrT = lane ? lane->mId : 0;
    if (hlwWg && (LUtj3 > 0.0))
    {
      Lane *pwxLH = new Lane();
      pwxLH->mId = JQlrT + 1;
      pwxLH->mType = xb0Qj;
      pwxLH->mSurfLevel = 0;
      addChild(pwxLH, true);
      LaneWidth *lWidth = new LaneWidth();
      lWidth->mOffset = 0.0;
      lWidth->mA = LUtj3;
      lWidth->mB = 0.0;
      lWidth->mC = 0.0;
      lWidth->mD = 0.0;
      pwxLH
          ->addChild(lWidth);
    }
    if (xre3q && (bOPv4 > 0.0))
    {
      lane = reinterpret_cast<Lane *>(
          getLastChild(ODR_OPCODE_LANE));
      int SPciz = lane ? lane->mId : 0;
      Lane *pwxLH = new Lane();
      pwxLH->mId = SPciz - 1;
      pwxLH->mType = sjRyt;
      pwxLH->mSurfLevel = 0;
      addChild(pwxLH);
      LaneWidth *lWidth = new LaneWidth();
      lWidth->mOffset = 0.0;
      lWidth->mA = bOPv4;
      lWidth->mB = 0.0;
      lWidth->mC = 0.0;
      lWidth->mD = 0.0;
      pwxLH->addChild(lWidth);
    }
  }
  Node *LaneSection::
      getCopy(bool Mupxf)
  {
    Node *dzamm = new LaneSection(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
  void LaneSection::calcPrepareData()
  {
    static bool Y3vvh = false;
    if (0 &&
        Y3vvh)
      fprintf(stderr,
              "\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x63\x61\x6c\x6c\x65\x64\x2e\x20\x4d\x4d\x4d\x4d\x4d"
              "\n");
    sortLanes();
    bool rgMav = false;
    bool c6B8w = false;
    Lane *lane = reinterpret_cast<Lane *>(getChild());
    Lane *evPNv = 0;
    while (lane)
    {
      rgMav |= lane->getChild(
                   ODR_OPCODE_LANE_BORDER) != 0;
      c6B8w |= lane->getChild(ODR_OPCODE_LANE_WIDTH) != 0;
      if (
          lane->mId == 0)
        evPNv = lane;
      else if ((abs(lane->mId) == 1) && !(mSideMask &
                                          sLaneSecSideFlagCenter))
        evPNv = lane;
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    if (rgMav && c6B8w && evPNv)
    {
      if (Y3vvh)
        fprintf(stderr,
                "\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x4d\x4d\x4d\x4d\x4d\x20\x6c\x61\x6e\x65\x20\x73\x65\x63\x74\x69\x6f\x6e\x20\x5b\x20\x25\x2e\x33\x6c\x66\x20\x3b\x20\x25\x2e\x33\x6c\x66\x20\x5d\x20\x68\x61\x73\x20\x6d\x69\x78\x65\x64\x20\x64\x65\x66\x69\x6e\x69\x74\x69\x6f\x6e"
                "\n",
                mS, mSEnd);
      for (int i = 0; i < 2; i++)
      {
        Lane *kY6_V = 0;
        lane = evPNv;
        while (lane)
        {
          if (Y3vvh)
            fprintf(stderr,
                    "\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x77\x6f\x72\x6b\x69\x6e\x67\x20\x6f\x6e\x20\x6c\x61\x6e\x65\x20\x25\x64"
                    "\n",
                    lane->mId);
          LaneWidth *qfK7D = lane->getFirstWidth();
          while (qfK7D)
          {
            if (Y3vvh)
              fprintf(
                  stderr,
                  "\x4c\x61\x6e\x65\x53\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x63\x6f\x6e\x76\x65\x72\x74\x69\x6e\x67\x20\x6c\x61\x6e\x65\x20\x77\x69\x64\x74\x68\x20\x5b\x20\x25\x2e\x33\x6c\x66\x20\x3b\x20\x25\x2e\x33\x6c\x66\x20\x5d\x20\x74\x6f\x20\x62\x6f\x72\x64\x65\x72"
                  "\n",
                  qfK7D->mOffset, qfK7D->mOffsetEnd);
            bool CypHo = lane->mId < 0;
            LaneBorder *i4O_J = new LaneBorder();
            i4O_J->mOffset = qfK7D->mOffset;
            i4O_J->mOffsetEnd = qfK7D->mOffsetEnd;
            i4O_J->mA = (CypHo ? -1.0 : 1.0) * qfK7D->mA;
            i4O_J->mB = (CypHo ? -1.0 : 1.0) * qfK7D->mB;
            i4O_J
                ->mC = (CypHo ? -1.0 : 1.0) * qfK7D->mC;
            i4O_J->mD = (CypHo ? -1.0 : 1.0) * qfK7D->mD;
            if (kY6_V)
            {
              LaneBorder *earcr = kY6_V->getFirstBorder();
              if (earcr)
              {
                i4O_J->mA += earcr->mA;
                i4O_J->mB += earcr->mB;
                i4O_J->mC += earcr->mC;
                i4O_J->mD += earcr->mD;
              }
            }
            lane->addChild(i4O_J);
            delete qfK7D;
            qfK7D = lane->getFirstWidth();
          }
          kY6_V = lane;
          if (i)
            lane = reinterpret_cast<
                Lane *>(lane->getRight());
          else
            lane = reinterpret_cast<Lane *>(lane->getLeft());
        }
      }
      if (Y3vvh)
        print(true);
    }
    if (mSideMask != sLaneSecSideFlagAll)
    {
      Lane *lane =
          reinterpret_cast<Lane *>(getChild());
      while (lane)
      {
        if (lane->mId < 0)
          mSideMask |=
              sLaneSecSideFlagRight;
        else if (lane->mId > 0)
          mSideMask |= sLaneSecSideFlagLeft;
        else
          mSideMask |= sLaneSecSideFlagCenter;
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
    }
  }
  void LaneSection::calcPostPrepareData()
  {
    double LUtj3 = 0.0;
    double bOPv4 = 0.0;
    Lane *lane = reinterpret_cast<Lane *>(getChild());
    while (lane)
    {
      double MMtQD;
      double
          wMax;
      lane->getWidthMinMax(MMtQD, wMax);
      if (lane->mId > 0)
        LUtj3 += wMax;
      else if (lane->mId < 0)
        bOPv4 += wMax;
      else
      {
        bOPv4 += 0.5 * wMax;
        LUtj3 += 0.5 * wMax;
      }
      lane = reinterpret_cast<
          Lane *>(lane->getRight());
    }
    mMaxHalfWidth = LUtj3 < bOPv4 ? bOPv4 : LUtj3;
    LaneSection *
        BsXtS = reinterpret_cast<LaneSection *>(getRight());
    while (BsXtS)
    {
      if ((BsXtS->mSideMask & mSideMask) == mSideMask)
      {
        if (mSEnd != BsXtS->mS)
        {
          mSEnd = BsXtS->mS;
          unsigned int hFNbl = 0;
          Node *rkAEh = 0;
          while ((rkAEh = getChildAtTypeIdx(hFNbl++)) != 0)
            rkAEh->prepare(true);
        }
        break;
      }
      BsXtS = reinterpret_cast<LaneSection *>(BsXtS->getRight());
    }
  }
  const double &LaneSection::getMaxHalfWidth() { return mMaxHalfWidth; }
  void
  LaneSection::sortLanes()
  {
    int DvSTt = 0;
    Lane *lane = reinterpret_cast<Lane *>(getChild());
    while (lane)
    {
      DvSTt++;
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    if (DvSTt >= 2)
    {
      for (int i = 0; i < DvSTt - 1; i++)
      {
        lane = reinterpret_cast<Lane *>(getChild());
        Lane *
            xYbZH = reinterpret_cast<Lane *>(lane->getRight());
        while (xYbZH)
        {
          if (xYbZH->mId > lane
                               ->mId)
            lane->shiftRight();
          else
            lane = xYbZH;
          xYbZH = reinterpret_cast<Lane *>(lane->getRight());
        }
      }
    }
  }
  bool LaneSection::containsSide(const int &laneId)
  {
    if (laneId < 0)
      return (mSideMask & sLaneSecSideFlagRight) != 0;
    if (laneId > 0)
      return (mSideMask &
              sLaneSecSideFlagLeft) != 0;
    if (laneId == 0)
      return (mSideMask & sLaneSecSideFlagCenter) !=
             0;
    return false;
  }
} // namespace OpenDrive
