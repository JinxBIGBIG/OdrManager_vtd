
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#ifdef LINUX
#include <unistd.h>
#else
#include <io.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "OdrReader.hh"
#include "OpenDRIVE.hh"
#include "OdrAllNodes.hh"
namespace OpenDrive
{
  const unsigned int Reader::FLAG_ADD_ROADS = 0x00000001;
  const unsigned int Reader::FLAG_REPLACE_ROADS = 0x00000002;
  const unsigned int Reader::
      FLAG_ADD_SIGNALS = 0x00000004;
  const unsigned int Reader::FLAG_SPLIT_JUNCTIONS =
      0x00000008;
  const unsigned int Reader::FLAG_CHECK_GEOMETRY = 0x00000010;
  const unsigned int Reader::NyNgk = 0x00000020;
  const unsigned int Reader::
      FLAG_INHIBIT_CURVATURE_APPROXIMATION = 0x00000040;
  Reader::Reader() : mDescriptor(-1), mEof(false), mRootNode(NULL), mVerbose(0), mAddBorderLanes(false), mBorderWidth(0.0), mFlags(0) {}
  Reader::~Reader() {}
  void Reader::setFilename(const std::string &LiJIF) { mFilename = LiJIF; }
  bool Reader::read()
  {
    mDescriptor = ::open(mFilename.c_str(),
                         O_RDONLY);
    if (mDescriptor < 0)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x72\x65\x61\x64\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x6f\x70\x65\x6e\x20\x66\x69\x6c\x65\x20\x3c"
                << mFilename << "\x3e\x2e" << std::endl;
      return false;
    }
    mRootNode = new UserData();
    parseFile();
    ::close(mDescriptor);
    return true;
  }
  bool Reader::add(const unsigned int)
  {
    std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x61\x64\x64\x3a\x20\x66\x75\x6e\x63\x74\x69\x6f\x6e\x61\x6c\x69\x74\x79\x20\x6e\x6f\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x69\x6e\x20\x62\x61\x73\x65\x20\x63\x6c\x61\x73\x73"
              << std::endl;
    return false;
  }
  int Reader::getDescriptor() const { return mDescriptor; }
  unsigned char Reader::getUChar()
  {
    unsigned char retVal;
    mEof = ::read(mDescriptor, &retVal, 1) < 1;
    return retVal;
  }
  unsigned short Reader::getUShort()
  {
    unsigned short
        retVal;
    mEof = ::read(mDescriptor, &retVal, 2) < 1;
    return retVal;
  }
  int Reader::getInt()
  {
    int retVal;
    mEof = ::read(mDescriptor, &retVal, 4) < 1;
    return retVal;
  }
  unsigned int
  Reader::getUInt()
  {
    unsigned int retVal;
    mEof = ::read(mDescriptor, &retVal, 4) < 1;
    return retVal;
  }
  double Reader::getDouble()
  {
    double retVal;
    mEof = ::read(mDescriptor,
                  &retVal, 8) < 1;
    return retVal;
  }
  std::string Reader::getString(const int length)
  {
    char
        *l0rb0 = (char *)calloc(length + 1, sizeof(char));
    mEof = ::read(mDescriptor, l0rb0, length) < length;
    std::string retVal(l0rb0);
    if (l0rb0)
      free(l0rb0);
    return retVal;
  }
  void *
  Reader::getRawData(const int length)
  {
    void *retVal = calloc(length, 1);
    if (retVal)
      mEof = ::read(mDescriptor, retVal, length) < length;
    return retVal;
  }
  void Reader::print()
  {
    if (!mRootNode)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x6e\x6f\x20\x64\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x21"
                << std::endl;
      return;
    }
    std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x70\x72\x69\x6e\x74\x3a\x20\x63\x6f\x6d\x70\x6c\x65\x74\x65\x20\x64\x61\x74\x61\x3a\x20"
              << std::endl;
    if (mRootNode)
      mRootNode->print();
  }
  void Reader::parseData(
      ParserCallback *dSmeZ)
  {
    if (!mRootNode)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x70\x61\x72\x73\x65\x44\x61\x74\x61\x3a\x20\x6e\x6f\x20\x64\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x21"
                << std::endl;
      return;
    }
    if (!dSmeZ)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x70\x61\x72\x73\x65\x44\x61\x74\x61\x3a\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x61\x72\x73\x65\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x63\x61\x6c\x6c\x62\x61\x63\x6b\x20\x64\x65\x66\x69\x6e\x65\x64\x21"
                << std::endl;
      return;
    }
    if (mRootNode)
      mRootNode->parse(dSmeZ);
    dSmeZ->endOfData();
  }
  Node *Reader::getRootNode() const { return mRootNode; }
  void Reader::setVerbose(
      unsigned int M8R_J) { mVerbose = M8R_J; }
  void Reader::addBorderLanes(const double &
                                  width)
  {
    if (width <= 0.0)
    {
      mAddBorderLanes = false;
      return;
    }
    mBorderWidth = width;
    mAddBorderLanes = true;
  }
  void Reader::splitJunctions()
  {
    if (mVerbose)
      std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x65\x6e\x74\x65\x72\x65\x64\x21"
                << std::endl;
    Node *node = mRootNode;
    RoadHeader *lPWHS = 0;
    if (!node)
    {
      if (mVerbose)
        std::
                cerr
            << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x6e\x6f\x20\x64\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x21"
            << std::endl;
      return;
    }
    RoadHeader *W1MOr = reinterpret_cast<RoadHeader *>(mRootNode->getChild(ODR_OPCODE_ROAD_HEADER));
    unsigned int _SmN_ = 0;
    while (W1MOr)
    {
      if (W1MOr->mId > _SmN_)
        _SmN_ = W1MOr->mId;
      W1MOr = reinterpret_cast<RoadHeader *>(W1MOr->getRight());
    }
    _SmN_++;
    node = node->getChild(ODR_OPCODE_JUNCTION_HEADER);
    while (node)
    {
      if (
          mVerbose)
      {
        std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x66\x6f\x75\x6e\x64\x20\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x20"
                  << std::endl;
        node->print(true, false);
      }
      JuncLink *hYfdO = reinterpret_cast<JuncLink *>(
          node->getChild(ODR_OPCODE_JUNCTION_LINK));
      while (hYfdO)
      {
        RoadHeader *japsW = hYfdO->mConnectingRoad;
        if (japsW && japsW->getNoDriveableLanes() > 1)
        {
          LaneSection *H4prs =
              japsW->getFirstLaneSection();
          if (H4prs)
          {
            Lane *lane = reinterpret_cast<Lane *>(H4prs->getChild(ODR_OPCODE_LANE));
            while (lane)
            {
              if (lane->isDriveable())
              {
                bool TS8kI = false;
                JuncLaneLink *SJBOF = reinterpret_cast<JuncLaneLink *>(hYfdO->getChild(
                    ODR_OPCODE_JUNCTION_LANE_LINK));
                while (SJBOF && !TS8kI)
                {
                  if (SJBOF->mPathLane == lane->mId)
                    TS8kI = true;
                  SJBOF = reinterpret_cast<JuncLaneLink *>(SJBOF->getRight());
                }
                if (!TS8kI)
                {
                  JuncLaneLink *Q288Y = new JuncLaneLink();
                  if (Q288Y)
                  {
                    Q288Y->mPathLane = lane->mId;
                    Q288Y->mRoadLane = 0;
                    hYfdO->addChild(Q288Y);
                    hYfdO->calcPrepareData();
                  }
                }
              }
              lane =
                  reinterpret_cast<Lane *>(lane->getRight());
            }
          }
          JuncLaneLink *SJBOF = reinterpret_cast<
              JuncLaneLink *>(hYfdO->getChild(ODR_OPCODE_JUNCTION_LANE_LINK));
          while (SJBOF)
          {
            Lane
                *V1F0K = japsW->getLaneInFirstSection(SJBOF->mPathLane);
            if (V1F0K && V1F0K->isDriveable())
              break;
            SJBOF = reinterpret_cast<JuncLaneLink *>(SJBOF->getRight());
          }
          JuncLaneLink *oY2EN = SJBOF;
          JuncLaneLink *X3oq0 = 0;
          if (SJBOF)
          {
            X3oq0 = reinterpret_cast<
                JuncLaneLink *>(SJBOF->getRight());
            while (X3oq0)
            {
              Lane *V1F0K = japsW->getLaneInFirstSection(X3oq0->mPathLane);
              if (V1F0K && V1F0K->isDriveable())
                break;
              X3oq0 = reinterpret_cast<JuncLaneLink *>(X3oq0->getRight());
            }
          }
          if (SJBOF && X3oq0)
          {
            SJBOF = X3oq0;
            while (SJBOF)
            {
              X3oq0 = reinterpret_cast<JuncLaneLink *>(SJBOF->getRight());
              while (X3oq0)
              {
                Lane *V1F0K = japsW->getLaneInFirstSection(X3oq0->mPathLane);
                if (
                    V1F0K && V1F0K->isDriveable())
                  break;
                X3oq0 = reinterpret_cast<JuncLaneLink *>(X3oq0->getRight());
              }
              RoadHeader *oSyrL = reinterpret_cast<RoadHeader *>(japsW->getCopy(true));
              if (!oSyrL)
              {
                fprintf(stderr,
                        "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x63\x72\x65\x61\x74\x65\x20\x61\x20\x63\x6f\x70\x79\x20\x6f\x66\x20\x70\x61\x74\x68\x20\x25\x70"
                        "\n",
                        japsW);
                return;
              }
              oSyrL->mId = _SmN_;
              char K6hUm[64];
              sprintf(K6hUm, "\x25\x64", oSyrL->mId);
              oSyrL->mIdAsString = std::string(K6hUm);
              japsW->getParent()->addChild(oSyrL);
              if (!lPWHS)
                lPWHS = oSyrL;
              oSyrL->invalidateLanes(SJBOF->mPathLane);
              if (mVerbose)
              {
                std ::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x63\x6f\x70\x69\x65\x64\x20\x70\x61\x74\x68\x3a\x20"
                           << std::endl;
                oSyrL->print();
              }
              if (SJBOF->mRoadLane == 0)
              {
                SJBOF->isolate();
              }
              else
              {
                JuncLink *ST1g8 = new JuncLink(hYfdO);
                ST1g8->mPathId = oSyrL->mId;
                ST1g8->mPathIdAsString = oSyrL->mIdAsString;
                node->addChild(ST1g8);
                SJBOF->isolate();
                ST1g8
                    ->addChild(SJBOF);
                ST1g8->calcPrepareData();
              }
              _SmN_++;
              SJBOF = X3oq0;
            }
            japsW->invalidateLanes(oY2EN->mPathLane, true);
          }
        }
        hYfdO = reinterpret_cast<JuncLink *>(hYfdO
                                                 ->getRight());
      }
      if (mVerbose)
      {
        std::cerr << "\x52\x65\x61\x64\x65\x72\x3a\x3a\x73\x70\x6c\x69\x74\x4a\x75\x6e\x63\x74\x69\x6f\x6e\x73\x3a\x20\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x20\x61\x66\x74\x65\x72\x20\x6d\x6f\x64\x69\x66\x69\x63\x61\x74\x69\x6f\x6e\x20"
                  << std::endl;
        node->print(true, false);
      }
      node = node->getRight();
    }
    if (lPWHS)
    {
      lPWHS->prepare();
    }
    convertClonedSignals();
  }
  void Reader::convertClonedSignals()
  {
    RoadHeader *W1MOr = reinterpret_cast<RoadHeader *>(mRootNode->getChild(
        ODR_OPCODE_ROAD_HEADER));
    while (W1MOr)
    {
      if (W1MOr->isCloned())
      {
        Signal *PERPi = W1MOr->getFirstSignal();
        while (PERPi)
        {
          SignalRef *fD5F_ = new SignalRef(PERPi);
          W1MOr->addChild(fD5F_);
          fD5F_->calcPrepareData();
          Signal *kjwcW = reinterpret_cast<Signal *>(
              PERPi->getRight());
          delete PERPi;
          PERPi = kjwcW;
        }
      }
      W1MOr = reinterpret_cast<RoadHeader *>(W1MOr->getRight());
    }
  }
  void Reader::addFlag(const unsigned long &flag) { mFlags |=
                                                    flag; }
} // namespace OpenDrive
