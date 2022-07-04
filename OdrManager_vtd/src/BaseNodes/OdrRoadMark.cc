
#include "OdrRoadMark.hh"
#include "OdrLane.hh"
#include "OdrLaneSection.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive
{
  // added by JM -- 20211102
  double RoadMark::getOffset() const {
    return mOffset;
  }
  unsigned short RoadMark::getType() const {
    return mType;
  }
  unsigned short RoadMark::getWeight() const {
    return mWeight;
  }
  unsigned short RoadMark::getColor() const {
    return mColor;
  }
  double RoadMark::getWidth() const {
    return mWidth;
  }


  RoadMark::RoadMark() : Node("\x52\x6f\x61\x64\x4d\x61\x72\x6b")
  {
    mOpcode = ODR_OPCODE_LANE_ROAD_MARK;
    mLevel = 3;
    mOffset = 0.0;
    mType = 0;
    mWeight = 0;
    mColor = 0;
    mWidth = 0.0;
    mHeight = 0.02;
    mS = 0.0;
    mSEnd = 0.0;
    mMaterial = std::string(
        "\x73\x74\x61\x6e\x64\x61\x72\x64");
  }
  RoadMark::RoadMark(RoadMark *Nf2Ao) : Node(
                                            Nf2Ao)
  {
    mOffset = Nf2Ao->mOffset;
    mType = Nf2Ao->mType;
    mWeight = Nf2Ao->mWeight;
    mColor =
        Nf2Ao->mColor;
    mWidth = Nf2Ao->mWidth;
    mMaterial = Nf2Ao->mMaterial;
    mHeight = Nf2Ao->mHeight;
    mS = Nf2Ao->mS;
    mSEnd = Nf2Ao->mSEnd;
  }
  RoadMark::~RoadMark() {}
  void RoadMark::
      printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mOffset);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x74\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mType);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x77\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x64"
            "\n",
            mWeight);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x63\x6f\x6c\x6f\x72\x3a\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mColor);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x20\x20\x25\x2e\x33\x6c\x66"
            "\n",
            mWidth);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x68\x65\x69\x67\x68\x74\x3a\x20\x20\x20\x20\x25\x2e\x33\x6c\x66"
            "\n",
            mHeight);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x6d\x61\x74\x65\x72\x69\x61\x6c\x3a\x20\x20\x25\x73"
            "\n",
            mMaterial.c_str());
  }
  bool RoadMark::read(ReaderXML *F3vnM)
  {
    mOffset = F3vnM->getDouble("\x73\x4f\x66\x66\x73\x65\x74");
    mType = F3vnM->getOpcodeFromRoadMarkType(F3vnM->getString("\x74\x79\x70\x65"));
    mWeight = F3vnM->getOpcodeFromRoadMarkWeight(F3vnM->getString("\x77\x65\x69\x67\x68\x74"));
    mColor = F3vnM->getOpcodeFromRoadMarkColor(F3vnM->getString("\x63\x6f\x6c\x6f\x72"));
    mWidth = F3vnM->getDouble("\x77\x69\x64\x74\x68");
    mHeight = F3vnM->getDouble(
        "\x68\x65\x69\x67\x68\x74");
    mMaterial = F3vnM->getString(
        "\x6d\x61\x74\x65\x72\x69\x61\x6c");
    return true;
  }
  Node *RoadMark::getCopy(bool
                              Mupxf)
  {
    Node *dzamm = new RoadMark(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
  void
  RoadMark::calcPrepareData()
  {
    LaneSection *H4prs = reinterpret_cast<LaneSection *>(
        getParent()->getParent());
    if (!H4prs)
      return;
    mS = H4prs->mS + mOffset;
    mSEnd = H4prs->mSEnd;
    RoadMark *t6V3J = reinterpret_cast<RoadMark *>(getLeft());
    if (t6V3J)
      t6V3J->mSEnd = mS;
  }
} // namespace OpenDrive
