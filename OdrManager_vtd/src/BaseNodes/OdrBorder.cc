
#include "OdrBorder.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive
{
  Border::Border() : Node("\x42\x6f\x72\x64\x65\x72")
  {
    mOpcode =
        ODR_OPCODE_BORDER;
    mLevel = 3;
  }
  Border::Border(Border *Nf2Ao) : Node(Nf2Ao)
  {
    mWidth =
        Nf2Ao->mWidth;
    mType = Nf2Ao->mType;
    mOutlineId = Nf2Ao->mOutlineId;
    mUseCompleteOutline = Nf2Ao->mUseCompleteOutline;
  }
  Border::~Border() {}
  void Border::
      printData() const
  {
    fprintf(stderr,
            "\x20\x20\x20\x20\x77\x69\x64\x74\x68\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x2e\x31\x30\x66"
            "\n",
            mWidth);
    fprintf(stderr,
            "\x20\x20\x20\x20\x74\x79\x70\x65\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x73"
            "\n",
            mType.c_str());
    fprintf(stderr,
            "\x20\x20\x20\x20\x6f\x75\x74\x6c\x69\x6e\x65\x49\x64\x3a\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x25\x64"
            "\n",
            mOutlineId);
    fprintf(stderr,
            "\x20\x20\x20\x20\x75\x73\x65\x43\x6f\x6d\x70\x6c\x65\x74\x65\x4f\x75\x74\x6c\x69\x6e\x65\x3a\x20\x25\x73"
            "\n",
            mUseCompleteOutline ? "\x74\x72\x75\x65" : "\x66\x61\x6c\x73\x65");
  }
  bool Border::
      read(ReaderXML *F3vnM)
  {
    mWidth = F3vnM->getDouble("\x77\x69\x64\x74\x68");
    mType =
        F3vnM->getString("\x74\x79\x70\x65");
    mOutlineId = F3vnM->getInt(
        "\x6f\x75\x74\x6c\x69\x6e\x65\x49\x64");
    mUseCompleteOutline = (F3vnM->getString(
                               "\x75\x73\x65\x43\x6f\x6d\x70\x6c\x65\x74\x65\x4f\x75\x74\x6c\x69\x6e\x65") ==
                           "\x66\x61\x6c\x73\x65")
                              ? false
                              : true;
    return true;
  }
  Node *Border::getCopy(bool Mupxf)
  {
    Node *dzamm = new Border(this);
    if (Mupxf)
      deepCopy(dzamm);
    return dzamm;
  }
} // namespace OpenDrive
