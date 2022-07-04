
#include "OdrControlEntry.hh"
#include <stdio.h>
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OdrSignal.hh"
namespace OpenDrive
{
  ControlEntry::ControlEntry() : Node(
                                     "\x43\x6f\x6e\x74\x72\x6f\x6c\x45\x6e\x74\x72\x79")
  {
    mOpcode =
        ODR_OPCODE_CONTROL_ENTRY;
    mLevel = 1;
    mSignal = (Signal *)0;
  }
  ControlEntry::~ControlEntry() {}
  void ControlEntry::printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x53\x69\x67\x6e\x61\x6c\x20\x49\x44\x3a\x20\x3c\x25\x73\x3e\x20\x25\x64"
            "\n",
            mSigIdAsString.c_str(), mSigId);
    fprintf(stderr,
            "\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mType);
    if (mSignal)
      fprintf(stderr,
              "\x20\x20\x20\x20\x53\x69\x67\x6e\x61\x6c\x3a\x20\x20\x20\x25\x70"
              "\n",
              mSignal);
  }
  bool ControlEntry::read(ReaderXML *F3vnM)
  {
    mSigId = F3vnM->getUInt(
        "\x73\x69\x67\x6e\x61\x6c\x49\x64");
    mSigIdAsString = F3vnM->getString(
        "\x73\x69\x67\x6e\x61\x6c\x49\x64");
    mType = F3vnM->getUInt("\x74\x79\x70\x65");
    return true;
  }
  void ControlEntry::calcPrepareData()
  {
    if (!RoadData::getInstance())
    {
      fprintf(stderr,
              "\x53\x69\x67\x6e\x61\x6c\x52\x65\x66\x3a\x3a\x63\x61\x6c\x63\x50\x72\x65\x70\x61\x72\x65\x44\x61\x74\x61\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21"
              "\n"
              "\x20");
      return;
    }
    Signal *PERPi = reinterpret_cast<Signal *>(RoadData::getInstance()->getNodeFromId(ODR_OPCODE_SIGNAL, mSigIdAsString));
    if (PERPi)
    {
      PERPi->setController(
          getParent());
      PERPi->setControlEntry(this);
      mSignal = PERPi;
    }
  }
} // namespace OpenDrive
