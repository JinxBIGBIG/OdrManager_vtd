
#include "OdrBridge.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive
{
  Bridge::Bridge() : Node("\x42\x72\x69\x64\x67\x65")
  {
    mOpcode =
        ODR_OPCODE_BRIDGE;
    mLevel = 1;
  }
  Bridge::Bridge(Bridge *Nf2Ao) : Node(Nf2Ao)
  {
    mS = Nf2Ao->mS;
    mLength = Nf2Ao->mLength;
    mName = Nf2Ao->mName;
    mId = Nf2Ao->mId;
    mIdAsString = Nf2Ao->mIdAsString;
    mType = Nf2Ao->mType;
  }
  Bridge::~Bridge() {}
  void Bridge::printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66"
            "\n",
            mS);
    fprintf(stderr,
            "\x20\x20\x20\x20\x64\x73\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66"
            "\n",
            mLength);
    fprintf(stderr,
            "\x20\x20\x20\x20\x4e\x61\x6d\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x73"
            "\n",
            mName.c_str());
    fprintf(stderr,
            "\x20\x20\x20\x20\x49\x44\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x3c\x25\x73\x3e\x20\x25\x64"
            "\n",
            mIdAsString.c_str(), mId);
    fprintf(stderr,
            "\x20\x20\x20\x20\x54\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mType);
  }
  bool Bridge::read(ReaderXML *F3vnM)
  {
    mS = F3vnM->getDouble("\x73");
    mLength =
        F3vnM->getDouble("\x6c\x65\x6e\x67\x74\x68");
    mName = F3vnM->getString(
        "\x6e\x61\x6d\x65");
    mId = F3vnM->getUInt("\x69\x64");
    mIdAsString = F3vnM->getString(
        "\x69\x64");
    mType = F3vnM->getOpcodeFromBridgeType(F3vnM->getString(
        "\x74\x79\x70\x65"));
    return true;
  }
  Node *Bridge::getCopy(bool Mupxf)
  {
    Node *dzamm =
        new Bridge(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
} // namespace OpenDrive
