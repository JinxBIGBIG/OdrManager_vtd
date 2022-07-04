
#include "OdrLaneWidth.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive
{
  LaneWidth::LaneWidth() : Node(
                               "\x4c\x61\x6e\x65\x57\x69\x64\x74\x68"),
                           mOffsetEnd(1.0e20), mMinWidth(0.0),
                           mMaxWidth(0.0)
  {
    mOpcode = ODR_OPCODE_LANE_WIDTH;
    mLevel = 3;
  }
  LaneWidth::LaneWidth(
      LaneWidth *Nf2Ao) : Node(Nf2Ao)
  {
    mOffset = Nf2Ao->mOffset;
    mA = Nf2Ao->mA;
    mB = Nf2Ao->mB;
    mC = Nf2Ao->mC;
    mD = Nf2Ao->mD;
    mOffsetEnd = Nf2Ao->mOffsetEnd;
    Nf2Ao->getMinMax(mMinWidth,
                     mMaxWidth);
  }
  LaneWidth::~LaneWidth() {}
  void LaneWidth::printData() const
  {
    fprintf(
        stderr,
        "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x4f\x66\x66\x73\x65\x74\x3a\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66\x20\x2d\x20\x25\x2e\x31\x30\x6c\x66"
        "\n",
        mOffset, mOffsetEnd);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x41\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mA);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x42\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mB);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x43\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mC);
    fprintf(stderr,
            "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x6c\x66"
            "\n",
            mD);
  }
  bool LaneWidth::read(ReaderXML *F3vnM)
  {
    mOffset = F3vnM->getDouble(
        "\x73\x4f\x66\x66\x73\x65\x74");
    mA = F3vnM->getDouble("\x61");
    mB = F3vnM->getDouble(
        "\x62");
    mC = F3vnM->getDouble("\x63");
    mD = F3vnM->getDouble("\x64");
    LaneWidth *Mp7pR =
        reinterpret_cast<LaneWidth *>(getLeft());
    if (Mp7pR)
      Mp7pR->mOffsetEnd = mOffset;
    return true;
  }
  double LaneWidth::ds2width(const double &rganP)
  {
    double BJBDA = rganP -
                   mOffset;
    return mA + mB * BJBDA + mC * BJBDA * BJBDA + mD * BJBDA * BJBDA * BJBDA;
  }
  double LaneWidth ::ds2widthDot(const double &rganP)
  {
    double BJBDA = rganP - mOffset;
    return mB + 2.0 * mC * BJBDA + 3.0 * mD * BJBDA * BJBDA;
  }
  double LaneWidth::ds2Curv(const double &rganP)
  {
    double
        BJBDA = rganP - mOffset;
    double gm6Wu = mB + 2.0 * mC * BJBDA + 3.0 * mD * BJBDA * BJBDA;
    gm6Wu *= gm6Wu;
    gm6Wu += 1.0;
    gm6Wu = sqrt(gm6Wu);
    gm6Wu *= gm6Wu * gm6Wu;
    return (2.0 * mC + 6.0 * mD * BJBDA) /
           gm6Wu;
  }
  void LaneWidth::calcPrepareData()
  {
    LaneWidth *Gv01F = reinterpret_cast<
        LaneWidth *>(getRight());
    LaneSection *sec = reinterpret_cast<LaneSection *>(getParent()->getParent());
    mOffsetEnd = Gv01F ? Gv01F->mOffset : sec->mSEnd - sec->mS;
  }
  void
  LaneWidth::calcPostPrepareData()
  {
    double h3p6P = ds2width(mOffset);
    double msQkJ =
        ds2width(mOffsetEnd);
    mMinWidth = h3p6P;
    mMaxWidth = h3p6P;
    mMaxWidth = msQkJ > mMaxWidth ? msQkJ : mMaxWidth;
    mMinWidth = msQkJ < mMinWidth ? msQkJ : mMinWidth;
    if ((fabs(mD) < 1.0e-6) &&
        (fabs(mC) > 1.0e-6) && (fabs(mB) > 1.0e-6))
    {
      double BJBDA = -mB / (2.0 * mC);
      if ((BJBDA > 0.0) &&
          (BJBDA < (mOffsetEnd - mOffset)))
      {
        msQkJ = ds2width(mOffset + BJBDA);
        mMaxWidth = msQkJ >
                            mMaxWidth
                        ? msQkJ
                        : mMaxWidth;
        mMinWidth = msQkJ < mMinWidth ? msQkJ : mMinWidth;
      }
    }
  }
  Node *
  LaneWidth::getCopy(bool Mupxf)
  {
    Node *dzamm = new LaneWidth(this);
    if (Mupxf)
      deepCopy(
          dzamm);
    return dzamm;
  }
  void LaneWidth::getMinMax(double &hAogV, double &Fc2XK)
  {
    hAogV =
        mMinWidth;
    Fc2XK = mMaxWidth;
  }
} // namespace OpenDrive
