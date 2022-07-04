
#ifndef _OPENDRIVE_READER_HH
#define _OPENDRIVE_READER_HH
#include "BaseNodes/OdrNode.hh"
#include "OdrParserCallback.hh"
namespace OpenDrive
{
  class Reader
  {
  public:
    static const unsigned int FLAG_ADD_ROADS;
    static const unsigned int FLAG_REPLACE_ROADS;
    static const unsigned int FLAG_ADD_SIGNALS;
    static const unsigned int FLAG_SPLIT_JUNCTIONS;
    static const unsigned int FLAG_CHECK_GEOMETRY;
    static const unsigned int NyNgk;
    static const unsigned int FLAG_INHIBIT_CURVATURE_APPROXIMATION;
  protected:
    std::string mFilename;
    int mDescriptor;
    bool mEof;
    Node *mRootNode;
    Node *mLastNode;
    unsigned int mVerbose;
    bool mAddBorderLanes;
    double mBorderWidth;
    unsigned int mFlags;
  public:
    explicit Reader();
    virtual ~Reader();
    void setFilename(const std::string &LiJIF);
    virtual bool read();
    virtual bool add(const unsigned int flags);
    int getDescriptor() const;
    virtual bool parseFile(void *HLfEO = 0, bool UmrFZ = false, Node *SGrXV = 0) = 0;
    unsigned char getUChar();
    unsigned short getUShort();
    int getInt();
    unsigned int getUInt();
    double getDouble();
    std::string getString(const int length);
    void * getRawData(const int length);
    void print();
    void parseData(ParserCallback *dSmeZ);
    Node *getRootNode() const;
    void setVerbose(unsigned int M8R_J);
    void addBorderLanes(
    const double &width);
    void splitJunctions();
    void convertClonedSignals();
    void addFlag(const unsigned long &flag);
  };
} // namespace OpenDrive
#endif
