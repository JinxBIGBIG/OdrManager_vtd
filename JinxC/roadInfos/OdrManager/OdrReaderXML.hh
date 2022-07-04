
#ifndef _OPENDRIVE_READERXML_HH
#define _OPENDRIVE_READERXML_HH
#include "OdrReader.hh"
namespace OpenDrive
{
  class ReaderXML : public Reader
  {
  public:
    typedef std::map<std::
                         string,
                     std::string>
        AttribMap;
    const AttribMap &getAllAttributes() const { return mAttribMap; }
  private:
    typedef std::map<std::string, unsigned int> StringValMap;
    StringValMap mOpcodeMap;
    StringValMap mLaneTypeMap;
    StringValMap mRoadMarkTypeMap;
    StringValMap mRoadMarkWeightMap;
    StringValMap mRoadMarkColorMap;
    StringValMap
        mLaneAccessMap;
    StringValMap mTunnelTypeMap;
    StringValMap mBridgeTypeMap;
    StringValMap mCountryMap;
    StringValMap mRoadTypeMap;
    StringValMap mUnitMap;
    StringValMap mRoadMarkRuleMap;
    StringValMap QUzzb;
    StringValMap hYi15;
    Node *
        mRoadHeader;
    Node *mGeoHeader;
    Node *mLaneSec;
    Node *mLane;
    Node *mObject;
    Node *mSignal;
    Node *mTunBrd;
    Node *mController;
    Node *mJuncHeader;
    Node *mJuncLink;
    Node *mLastNewNode;
    Node *mRoadType;
    Node *mHeader;
    Node *mJuncGroup;
    Node *mRoadMark;
    Node *mRoadMarkType;
    Node *mParkingSpace;
    Node *mRailroadSwitch;
    Node *mOffset;
    Node *O1eZ7;
    Node *UriUj;
    Node *
        FN8fF;
    Node *NnwlR;
    Node *HFgoM;
    AttribMap mAttribMap;
    bool mAddMode;
    unsigned int
        mNoModif;
    std::string mLcString;
    bool mIsRailroad;
    float mContObjTesselation;
    bool
        mResolveRepeatedObjects;
    bool DhlWv;
    bool mDLp2;
  public:
    explicit ReaderXML();
    virtual ~ReaderXML();
    virtual bool read(const std::string &AZJf9);
    virtual bool read();
    virtual bool add(const unsigned int flags);
    virtual bool parseFile(void *HLfEO,
                           bool UmrFZ = false, Node *SGrXV = 0);
    unsigned char getUChar(const std::string &Gqdbt);
    unsigned short getUShort(const std::string &Gqdbt);
    int getInt(const std::string &
                   Gqdbt);
    unsigned int getUInt(const std::string &Gqdbt);
    double getDouble(const std ::string &Gqdbt);
    std::string getString(const std::string &Gqdbt);
    unsigned int
    getOpcodeFromLaneType(const std::string &name);
    unsigned int
    getOpcodeFromRoadMarkType(const std::string &name);
    unsigned int
    getOpcodeFromRoadMarkWeight(const std::string &name);
    unsigned int
    getOpcodeFromRoadMarkColor(const std::string &name);
    unsigned int
    getOpcodeFromLaneAccess(const std::string &name);
    unsigned int
    getOpcodeFromTunnelType(const std::string &name);
    unsigned int
    getOpcodeFromBridgeType(const std::string &name);
    unsigned int
    getOpcodeFromCountry(const std::string &name);
    unsigned int getOpcodeFromRoadType(
        const std::string &name);
    unsigned int getOpcodeFromUnit(const std::string &name);
    unsigned int getOpcodeFromRoadMarkRule(const std::string &name);
    unsigned int
    CgGZS(const std::string &name);
    unsigned int zeDa8(const std::string &name);
    bool
    hasAttribute(const std::string &Gqdbt);
    void SrdMD(bool F744N);
    void Fa4eH(const float &value);
  private:
    bool parseGenericNode(void *HLfEO, Node &XFL9o);
    unsigned int
    getOpcodeFromElement(const std::string &name);
    void initOpcodes();
    void
    addAttribute(const std::string &key, const std::string &value);
    void clearAttributes();
    void prepareMap(StringValMap &wVomq);
    const std::string &loCase(const std::
                                  string &name);
    void A3Ua9();
  };
} // namespace OpenDrive
#endif
