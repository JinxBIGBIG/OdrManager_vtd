
#include "OdrRoadLink.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive
{
  RoadLink::RoadLink() : Node("\x52\x6f\x61\x64\x4c\x69\x6e\x6b")
  {
    mOpcode = ODR_OPCODE_ROAD_LINK;
    mLevel = 1;
  }

  RoadLink::RoadLink(RoadLink *Nf2Ao) : Node(Nf2Ao)
  {
    mType = Nf2Ao->mType;
    mElemType = Nf2Ao->mElemType;
    mElemId = Nf2Ao->mElemId;
    mElemIdAsString = Nf2Ao->mElemIdAsString;
    mElemDir = Nf2Ao->mElemDir;
    mSide = Nf2Ao->mSide;
    mElemS = Nf2Ao->mElemS;
    mVirtualLink = Nf2Ao->mVirtualLink;
  }

  unsigned int RoadLink::getElemId() const {
    return mElemId;
  }

  unsigned short RoadLink::getElemType() const {
    return mElemType;
  }

  std::string RoadLink::getElemIdStr() const {
    return mElemIdAsString;
  }

  RoadLink::RoadLink(unsigned short type) : Node("\x52\x6f\x61\x64\x4c\x69\x6e\x6b")
  {
    mOpcode =
        ODR_OPCODE_ROAD_LINK;
    mLevel = 1;
    mType = type;
  }

  RoadLink::~RoadLink() {}
  void RoadLink::printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mType);
    fprintf(stderr,
            "\x20\x20\x20\x20\x45\x6c\x65\x6d\x2e\x20\x54\x79\x70\x65\x3a\x20\x25\x64"
            "\n",
            mElemType);
    fprintf(stderr,
            "\x20\x20\x20\x20\x45\x6c\x65\x6d\x2e\x20\x49\x44\x3a\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64"
            "\n",
            mElemIdAsString.c_str(), mElemId);
    fprintf(stderr,
            "\x20\x20\x20\x20\x45\x6c\x65\x6d\x2e\x20\x44\x69\x72\x3a\x20\x20\x25\x64"
            "\n",
            mElemDir);
    if (mType == ODR_LINK_TYPE_NEIGHBOR)
      fprintf(stderr,
              "\x20\x20\x20\x20\x73\x69\x64\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x25\x64"
              "\n",
              mSide);
  }

  bool RoadLink::read(ReaderXML *F3vnM)
  {
    mElemType = (F3vnM->getString(
                     "\x65\x6c\x65\x6d\x65\x6e\x74\x54\x79\x70\x65") == "\x72\x6f\x61\x64")
                    ? ODR_TYPE_ROAD
                    : ODR_TYPE_JUNCTION;
    mElemId = F3vnM->getUInt(
        "\x65\x6c\x65\x6d\x65\x6e\x74\x49\x64");
    mElemIdAsString = F3vnM->getString(
        "\x65\x6c\x65\x6d\x65\x6e\x74\x49\x64");
    mVirtualLink = false;
    if (mType ==
        ODR_LINK_TYPE_NEIGHBOR)
    {
      if (F3vnM->getString(
              "\x64\x69\x72\x65\x63\x74\x69\x6f\x6e") == "\x73\x61\x6d\x65")
        mElemDir =
            ODR_DIRECTION_SAME;
      else if (F3vnM->getString(
                   "\x64\x69\x72\x65\x63\x74\x69\x6f\x6e") == "\x6f\x70\x70\x6f\x73\x69\x74\x65")
        mElemDir = ODR_DIRECTION_OPPOSITE;
      else
        mElemDir = ODR_DIRECTION_NONE;
      if (F3vnM->getString("\x73\x69\x64\x65") == "\x6c\x65\x66\x74")
        mSide = ODR_SIDE_LEFT;
      else if (
          F3vnM->getString("\x73\x69\x64\x65") == "\x72\x69\x67\x68\x74")
        mSide =
            ODR_SIDE_RIGHT;
      else
        mSide = ODR_SIDE_UNDEFINED;
    }
    else
    {
      if (F3vnM->getString(
              "\x63\x6f\x6e\x74\x61\x63\x74\x50\x6f\x69\x6e\x74") == "\x73\x74\x61\x72\x74")
        mElemDir = ODR_LINK_POINT_START;
      else if (F3vnM->getString(
                   "\x63\x6f\x6e\x74\x61\x63\x74\x50\x6f\x69\x6e\x74") == "\x65\x6e\x64")
        mElemDir =
            ODR_LINK_POINT_END;
      else if (!(F3vnM->getString(
                          "\x63\x6f\x6e\x74\x61\x63\x74\x50\x6f\x69\x6e\x74")
                     .length()) &&
               (mElemType ==
                ODR_TYPE_ROAD))
      {
        if (F3vnM->getString("\x65\x6c\x65\x6d\x65\x6e\x74\x53").length() && F3vnM->getString("\x65\x6c\x65\x6d\x65\x6e\x74\x44\x69\x72").length())
        {
          mElemS =
              F3vnM->getDouble("\x65\x6c\x65\x6d\x65\x6e\x74\x53");
          mElemDir = F3vnM->getString(
                         "\x65\x6c\x65\x6d\x65\x6e\x74\x44\x69\x72") == std::string("\x2d")
                         ? ODR_DIRECTION_MINUS
                         : ODR_DIRECTION_PLUS;
          mVirtualLink = true;
        }
        else
          mElemDir = 0;
      }
      else
        mElemDir = 0;
      mSide = ODR_SIDE_UNDEFINED;
    }
    return true;
  }
  
  Node *RoadLink::getCopy(bool
                              Mupxf)
  {
    Node *dzamm = new RoadLink(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
} // namespace OpenDrive
