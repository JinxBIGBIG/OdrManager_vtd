
#include <iostream>
#include <stdio.h>
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OpenDRIVE.hh"
#include "OdrAllNodes.hh"
#include "OdrGeoPoly.hh"
#include "OdrGeoParamPoly.hh"
#include "OdrGeoSpiralOdr.hh"
#include "OdrGenericNode.hh"
#include "tinyxml.h"
#include <math.h>
// #include <unistd.h>
namespace OpenDrive
{
  ReaderXML::ReaderXML() : mAddMode(false), mNoModif(0)
  {
    initOpcodes();
    mRoadHeader = 0;
    mGeoHeader = 0;
    mLaneSec = 0;
    mLane = 0;
    mObject = 0;
    O1eZ7 = 0;
    mSignal = 0;
    mTunBrd = 0;
    mController = 0;
    mJuncHeader = 0;
    mJuncLink = 0;
    mLastNewNode = 0;
    mResolveRepeatedObjects = true;
    mContObjTesselation = 2.0;
    mOffset = 0;
    O1eZ7 = 0;
    UriUj = 0;
    FN8fF = 0;
    NnwlR = 0;
  }
  ReaderXML::~ReaderXML() {}
  bool ReaderXML::read()
  {
    static bool
        Y3vvh = false;
    TiXmlDocument w0P3a(mFilename.c_str());
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x62\x65\x66\x6f\x72\x65\x20\x4c\x6f\x61\x64\x46\x69\x6c\x65\x28\x29"
              "\n");
    bool QdOVz = w0P3a.LoadFile();
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x61\x66\x74\x65\x72\x20\x4c\x6f\x61\x64\x46\x69\x6c\x65\x28\x29"
              "\n");
    if (!QdOVz)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x65\x72\x72\x6f\x72\x20\x6c\x6f\x61\x64\x69\x6e\x67\x20\x3c"
                << mFilename << "\x3e" << std::endl;
      return false;
    }
    mRootNode = new UserData();
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x73\x74\x61\x72\x74\x69\x6e\x67\x20\x70\x61\x72\x73\x65\x72"
              "\n");
    parseFile(&w0P3a);
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x61\x70\x70\x6c\x79\x69\x6e\x67\x20\x6f\x66\x66\x73\x65\x74\x73"
              "\n");
    A3Ua9();
    RoadData *Zrso9 = RoadData::getInstance();
    if (!Zrso9)
    {
      if (Y3vvh)
        fprintf(
            stderr,
            "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x63\x72\x65\x61\x74\x69\x6e\x67\x20\x72\x6f\x6f\x74\x20\x6e\x6f\x64\x65"
            "\n");
      Zrso9 = new RoadData(mRootNode);
    }
    else
    {
      if (Y3vvh)
        fprintf(stderr,
                "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x61\x73\x73\x69\x67\x6e\x69\x6e\x67\x20\x72\x6f\x6f\x74\x20\x6e\x6f\x64\x65"
                "\n");
      Zrso9->setRootNode(mRootNode);
    }
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x73\x74\x61\x72\x74\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x70\x72\x65\x70\x61\x72\x61\x74\x69\x6f\x6e\x73"
              "\n");
    mRootNode->prepare();
    if (Y3vvh)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x70\x72\x65\x70\x61\x72\x61\x74\x69\x6f\x6e\x20\x64\x6f\x6e\x65"
              "\n");
    if (mFlags & FLAG_SPLIT_JUNCTIONS)
      splitJunctions();
    return true;
  }
  bool ReaderXML::
      read(const std::string &AZJf9)
  {
    TiXmlDocument w0P3a;
    bool QdOVz = w0P3a.Parse((const char *)AZJf9.c_str(), 0, TIXML_ENCODING_UTF8) == 0;
    if (!QdOVz)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x72\x65\x61\x64\x3a\x20\x65\x72\x72\x6f\x72\x20\x72\x65\x61\x64\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x73\x74\x72\x69\x6e\x67\x2e"
                << std::endl;
      return false;
    }
    mRootNode = new UserData();
    parseFile(&w0P3a);
    mRootNode->prepare();
    if (mFlags & FLAG_SPLIT_JUNCTIONS)
      splitJunctions();
    return true;
  }
  bool
  ReaderXML::add(const unsigned int flags)
  {
    if (!(flags & FLAG_ADD_SIGNALS))
    {
      std::cerr
          << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x61\x64\x64\x3a\x20\x69\x6e\x76\x61\x6c\x69\x64\x20\x66\x6c\x61\x67\x20\x6f\x72\x20\x66\x6c\x61\x67\x20\x63\x6f\x6d\x62\x69\x6e\x61\x74\x69\x6f\x6e"
          << std::endl;
      return false;
    }
    mFlags = flags;
    if (!RoadData::getInstance())
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x61\x64\x64\x3a\x20\x4e\x6f\x20\x52\x6f\x61\x64\x44\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2e\x20\x54\x72\x79\x20\x72\x65\x61\x64\x28\x29\x20\x66\x69\x72\x73\x74\x2e"
                << std::endl;
      return false;
    }
    mRootNode = RoadData::getInstance()->getRootNode();
    if (!mRootNode)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x61\x64\x64\x3a\x20\x4e\x6f\x20\x64\x61\x74\x61\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2e\x20\x54\x72\x79\x20\x72\x65\x61\x64\x28\x29\x20\x66\x69\x72\x73\x74\x2e"
                << std::endl;
      return false;
    }
    TiXmlDocument w0P3a(mFilename.c_str());
    bool QdOVz =
        w0P3a.LoadFile();
    if (!QdOVz)
    {
      std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x61\x64\x64\x3a\x20\x65\x72\x72\x6f\x72\x20\x6c\x6f\x61\x64\x69\x6e\x67\x20\x3c"
                << mFilename << "\x3e" << std::endl;
      return false;
    }
    RoadData::getInstance()->incNoModifications();
    mNoModif = RoadData::getInstance()->getNoModifications();
    mAddMode = true;
    parseFile(&w0P3a);
    RoadData::getInstance()->buildIndexTable();
    mRootNode->prepare();
    RoadData::getInstance()->buildIndexTable();
    return true;
  }
  bool ReaderXML::parseFile(void *gE3Uy, bool UmrFZ, Node *SGrXV)
  {
    Node *dzamm = 0;
    TiXmlNode *HLfEO = (TiXmlNode *)gE3Uy;
    TiXmlNode *iaLTA;
    TiXmlText *j2ESK;
    if (!HLfEO)
      return false;
    int rkiXc = HLfEO->Type();
    bool YXdS5 = true;
    if (rkiXc == TiXmlNode::ELEMENT)
    {
      unsigned int CvVIp = getOpcodeFromElement(HLfEO->Value());
      // if (!(HLfEO->Row() % 10000))
      //   usleep(1);
      if (mVerbose)
        std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x72\x65\x61\x64\x69\x6e\x67\x20\x65\x6c\x65\x6d\x65\x6e\x74\x20\x5b"
                  << HLfEO->Value() << "\x5d\x20\x77\x69\x74\x68\x20\x6f\x70\x63\x6f\x64\x65\x20" << CvVIp << "\x2e\x2e";
      dzamm = 0;
      if (!UmrFZ)
      {
        switch (CvVIp)
        {
        case ODR_OPCODE_ROAD_TYPE:
          if (
              mRoadMark)
            CvVIp = ODR_OPCODE_ROAD_MARK_TYPE;
          break;
        case ODR_OPCODE_GEO_LINE:
          if (
              mRoadMark)
          {
            if (DhlWv)
              CvVIp = ODR_OPCODE_ROAD_MARK_EXPLICIT_LINE;
            else
              CvVIp =
                  ODR_OPCODE_ROAD_MARK_LINE;
          }
          break;
        case ODR_OPCODE_LANE_SPEED:
          if (mRoadType && !mLane)
            CvVIp = ODR_OPCODE_ROAD_SPEED;
          break;
        case ODR_OPCODE_CONTROLLER:
          if (mJuncHeader)
            CvVIp = ODR_OPCODE_JUNCTION_CONTROL;
          break;
        case ODR_OPCODE_SWITCH:
          if (mRoadHeader &&
              mIsRailroad)
            CvVIp = ODR_OPCODE_RAILROAD_SWITCH;
          break;
        case ODR_OPCODE_MARKINGS_MARKING:
          if (mParkingSpace)
            CvVIp =
                ODR_OPCODE_PARKING_SPACE_MARKING;
          break;
        case ODR_OPCODE_PREDECESSOR:
          if (mJuncLink)
            CvVIp = ODR_OPCODE_JUNCTION_LINK_PREDECESSOR;
          break;
        case ODR_OPCODE_SUCCESSOR:
          if (
              mJuncLink)
            CvVIp = ODR_OPCODE_JUNCTION_LINK_SUCCESSOR;
          break;
        default:
          break;
        }
        switch (
            CvVIp)
        {
        case ODR_OPCODE_HEADER:
          dzamm = new Header();
          mHeader = dzamm;
          SGrXV = mRootNode;
          mRoadHeader = 0;
          break;
        case ODR_OPCODE_ROAD_HEADER:
          mRoadHeader = new RoadHeader();
          {
            RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mRoadHeader);
            M8Gux->mContObjTesselation = mContObjTesselation;
            M8Gux->mResolveRepeatedObjects =
                mResolveRepeatedObjects;
          }
          SGrXV = mRootNode;
          dzamm = mRoadHeader;
          mLane = 0;
          mSignal = 0;
          mRoadType = 0;
          mRoadMark = 0;
          mRoadMarkType = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          mObject =
              0;
          O1eZ7 = 0;
          NnwlR = 0;
          mDLp2 = 0;
          mIsRailroad = false;
          mRailroadSwitch = 0;
          DhlWv = false;
          break;
        case ODR_OPCODE_PREDECESSOR:
          if (mLane)
          {
            dzamm = new LaneLink(
                ODR_LINK_TYPE_PREDECESSOR);
            SGrXV = mLane;
          }
          else
          {
            dzamm = new RoadLink(
                ODR_LINK_TYPE_PREDECESSOR);
            SGrXV = mRoadHeader;
          }
          break;
        case ODR_OPCODE_SUCCESSOR:
          if (mLane)
          {
            dzamm = new LaneLink(ODR_LINK_TYPE_SUCCESSOR);
            SGrXV = mLane;
          }
          else
          {
            dzamm = new RoadLink(ODR_LINK_TYPE_SUCCESSOR);
            SGrXV = mRoadHeader;
          }
          break;
        case ODR_OPCODE_NEIGHBOR:
          dzamm = new RoadLink(ODR_LINK_TYPE_NEIGHBOR);
          SGrXV = mRoadHeader;
          break;
        case ODR_OPCODE_ROAD_TYPE:
          dzamm = new RoadType();
          SGrXV = mRoadHeader;
          mRoadType = dzamm;
          mLane = 0;
          break;
        case ODR_OPCODE_GEO_HEADER:
          mGeoHeader = new GeoHeader();
          {
            GeoHeader *ya7Vb = reinterpret_cast<GeoHeader *>(mGeoHeader);
            ya7Vb->mCheckGeometry = (mFlags & FLAG_CHECK_GEOMETRY) != 0;
            ya7Vb->mEnableCurvApprox = (mFlags &
                                        FLAG_INHIBIT_CURVATURE_APPROXIMATION) == 0;
          }
          SGrXV = mRoadHeader;
          dzamm = mGeoHeader;
          break;
        case ODR_OPCODE_GEO_LINE:
          dzamm = new GeoLine();
          SGrXV = mGeoHeader;
          break;
        case ODR_OPCODE_GEO_SPIRAL:
          dzamm = new GeoSpiral();
          SGrXV = mGeoHeader;
          break;
        case ODR_OPCODE_GEO_ARC:
          dzamm = new GeoArc();
          SGrXV = mGeoHeader;
          break;
        case ODR_OPCODE_GEO_POLY:
          dzamm = new GeoPoly();
          SGrXV = mGeoHeader;
          break;
        case ODR_OPCODE_GEO_PARAM_POLY:
          dzamm = new GeoParamPoly();
          SGrXV = mGeoHeader;
          break;
        case ODR_OPCODE_ELEVATION:
          dzamm = new Elevation();
          SGrXV = mRoadHeader;
          break;
        case ODR_OPCODE_CROSSFALL:
          dzamm = new Crossfall();
          SGrXV = mRoadHeader;
          break;
        case ODR_OPCODE_SUPERELEVATION:
          dzamm = new Superelevation();
          SGrXV = mRoadHeader;
          break;
        case ODR_OPCODE_LANE_SECTION:
          mLaneSec = new LaneSection();
          SGrXV = mRoadHeader;
          dzamm =
              mLaneSec;
          break;
        case ODR_OPCODE_LANE:
          mLane = new Lane();
          SGrXV = mLaneSec;
          dzamm = mLane;
          mRoadType = 0;
          mRoadMark = 0;
          DhlWv = false;
          break;
        case ODR_OPCODE_LANE_WIDTH:
          dzamm = new LaneWidth();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_LANE_MATERIAL:
          if (mObject)
          {
            dzamm =
                new ObjectMaterial();
            SGrXV = mObject;
          }
          else if (mLane)
          {
            dzamm = new LaneMaterial();
            SGrXV = mLane;
          }
          break;
        case ODR_OPCODE_LANE_VISIBILITY:
          dzamm = new LaneVisibility();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_LANE_ACCESS:
          dzamm = new LaneAccess();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_LANE_HEIGHT:
          dzamm = new LaneHeight();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_LANE_RULE:
          dzamm = new LaneRule();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_LANE_SPEED:
          dzamm = new LaneSpeed();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_ROAD_SPEED:
          dzamm = new RoadSpeed();
          SGrXV = mRoadType;
          break;
        case ODR_OPCODE_OBJECT:
          mObject = new Object();
          mSignal = 0;
          mTunBrd = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          O1eZ7 = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mObject;
          break;
        case ODR_OPCODE_TUNNEL:
          mTunBrd = new Tunnel();
          mObject = 0;
          mSignal = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          O1eZ7 = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mTunBrd;
          break;
        case ODR_OPCODE_BRIDGE:
          mTunBrd = new Bridge();
          mObject = 0;
          mSignal = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          O1eZ7 = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mTunBrd;
          break;
        case ODR_OPCODE_REPEAT:
          dzamm = new Repeat();
          if (mObject)
            SGrXV = mObject;
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x52\x65\x70\x65\x61\x74\x20\x62\x65\x61\x64\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_SIGNAL_REFERENCE:
          mSignal = new SignalRef();
          mObject = 0;
          mTunBrd = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          O1eZ7 = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mSignal;
          break;
        case ODR_OPCODE_LANE_VALIDITY:
          dzamm = new LaneValidity();
          if (mObject)
            SGrXV = mObject;
          else if (mSignal)
            SGrXV = mSignal;
          else if (
              mTunBrd)
            SGrXV = mTunBrd;
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x4c\x61\x6e\x65\x56\x61\x6c\x69\x64\x69\x74\x79\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x20\x6f\x72\x20\x73\x69\x67\x6e\x61\x6c\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_CORNER_INERTIAL:
          dzamm = new CornerInertial();
          if (
              mObject)
          {
            if (!O1eZ7)
            {
              O1eZ7 = new ObjectOutline();
              mObject->addChild(O1eZ7);
            }
            SGrXV =
                O1eZ7;
          }
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x43\x6f\x72\x6e\x65\x72\x49\x6e\x65\x72\x74\x69\x61\x6c\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_CORNER_ROAD:
          dzamm = new CornerRoad();
          if (mObject)
          {
            if (!O1eZ7)
            {
              O1eZ7 = new ObjectOutline();
              mObject->addChild(O1eZ7);
            }
            SGrXV = O1eZ7;
          }
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x43\x6f\x72\x6e\x65\x72\x52\x6f\x61\x64\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_CORNER_RELATIVE:
          dzamm = new CornerRelative();
          if (
              mObject)
          {
            if (!O1eZ7)
            {
              O1eZ7 = new ObjectOutline();
              mObject->addChild(O1eZ7);
            }
            SGrXV =
                O1eZ7;
          }
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x43\x6f\x72\x6e\x65\x72\x52\x65\x6c\x61\x74\x69\x76\x65\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_CORNER_LOCAL:
          dzamm = new CornerLocal();
          if (
              mObject)
          {
            if (!O1eZ7)
            {
              O1eZ7 = new ObjectOutline();
              mObject->addChild(O1eZ7);
            }
            SGrXV =
                O1eZ7;
          }
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x43\x6f\x72\x6e\x65\x72\x4c\x6f\x63\x61\x6c\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x6f\x62\x6a\x65\x63\x74\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_SIGNAL:
          mSignal = new Signal();
          mObject = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mSignal;
          break;
        case ODR_OPCODE_SIGNAL_DEPEND:
          dzamm = new SignalDep();
          if (mSignal)
            SGrXV =
                mSignal;
          else
            std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x53\x69\x67\x6e\x61\x6c\x44\x65\x70\x65\x6e\x64\x65\x6e\x63\x79\x20\x64\x65\x66\x69\x6e\x65\x64\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x70\x72\x65\x76\x69\x6f\x75\x73\x20\x73\x69\x67\x6e\x61\x6c\x21"
                      << std::endl;
          break;
        case ODR_OPCODE_CONTROLLER:
          mController = new Controller();
          SGrXV =
              mRootNode;
          dzamm = mController;
          break;
        case ODR_OPCODE_JUNCTION_CONTROL:
          dzamm = new JuncController();
          SGrXV = mJuncHeader;
          break;
        case ODR_OPCODE_CONTROL_ENTRY:
          dzamm = new ControlEntry();
          SGrXV = mController;
          break;
        case ODR_OPCODE_JUNCTION_HEADER:
          mJuncHeader = new JuncHeader();
          mRoadHeader = 0;
          SGrXV = mRootNode;
          dzamm = mJuncHeader;
          break;
        case ODR_OPCODE_JUNCTION_LINK:
          mJuncLink = new JuncLink();
          SGrXV = mJuncHeader;
          dzamm = mJuncLink;
          break;
        case ODR_OPCODE_JUNCTION_LANE_LINK:
          dzamm = new JuncLaneLink();
          SGrXV = mJuncLink;
          break;
        case ODR_OPCODE_JUNCTION_PRIORITY:
          dzamm = new JuncPriority();
          SGrXV = mJuncHeader;
          break;
        case ODR_OPCODE_LANE_ROAD_MARK:
          dzamm = new RoadMark();
          SGrXV = mLane;
          mRoadMark = dzamm;
          mRoadMarkType = 0;
          DhlWv = false;
          break;
        case ODR_OPCODE_ROAD_MARK_TYPE:
          dzamm = new RoadMarkType();
          SGrXV = mRoadMark;
          mRoadMarkType = dzamm;
          break;
        case ODR_OPCODE_ROAD_MARK_LINE:
          dzamm = new RoadMarkLine();
          SGrXV =
              mRoadMarkType;
          break;
        case ODR_OPCODE_ROAD_MARK_SWAY:
          dzamm = new RoadMarkSway();
          SGrXV = mRoadMark;
          break;
        case ODR_OPCODE_ROAD_MARK_EXPLICIT:
          dzamm = 0;
          SGrXV = 0;
          DhlWv =
              true;
          break;
        case ODR_OPCODE_ROAD_MARK_EXPLICIT_LINE:
          dzamm = new RoadMarkExplicitLine();
          SGrXV = mRoadMark;
          break;
        case ODR_OPCODE_USER_DATA:
          dzamm = new UserData();
          if (SGrXV)
            dzamm->setLevel(SGrXV->getLevel() + 1);
          YXdS5 = false;
          parseGenericNode(HLfEO, *dzamm);
          break;
        case ODR_OPCODE_LANE_OFFSET:
          dzamm = new LaneOffset();
          SGrXV = mRoadHeader;
          if (SGrXV)
            dzamm->setLevel(SGrXV->getLevel() + 1);
          break;
        case ODR_OPCODE_SURFACE_CRG:
          if (mFlags & NyNgk)
            SGrXV = 0;
          else
          {
            dzamm = new SurfaceCRG();
            if (dzamm)
            {
              SurfaceCRG *SgQ7l = (SurfaceCRG *)dzamm;
              SgQ7l->mParentFileName = mFilename;
            }
            if (mRoadHeader)
              SGrXV = mRoadHeader;
            else if (mJuncHeader)
              SGrXV = mJuncHeader;
          }
          break;
        case ODR_OPCODE_LANE_BORDER:
          dzamm = new LaneBorder();
          SGrXV = mLane;
          break;
        case ODR_OPCODE_GEO_REFERENCE:
          dzamm = new GeoReference();
          SGrXV =
              mHeader;
          break;
        case ODR_OPCODE_OFFSET:
          dzamm = new Offset();
          mOffset = dzamm;
          SGrXV =
              mHeader;
          break;
        case ODR_OPCODE_LATERAL_SHAPE:
          dzamm = new LateralShape();
          SGrXV =
              mRoadHeader;
          break;
        case ODR_OPCODE_JUNCTION_GROUP:
          dzamm = new JuncGroup();
          mJuncGroup = dzamm;
          SGrXV = mRootNode;
          break;
        case ODR_OPCODE_JUNCTION_REFERENCE:
          dzamm =
              new JuncRef();
          SGrXV = mJuncGroup;
          break;
        case ODR_OPCODE_PARKING_SPACE:
          if (!mObject)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new ParkingSpace();
          mParkingSpace = dzamm;
          SGrXV = mObject;
          break;
        case ODR_OPCODE_PARKING_SPACE_MARKING:
          if (!mParkingSpace)
          {
            SGrXV = 0;
            break;
          }
          dzamm =
              new ParkingSpaceMarking();
          SGrXV = mParkingSpace;
          break;
        case ODR_OPCODE_MARKINGS:
          if (
              !mObject)
          {
            SGrXV = 0;
            break;
          }
          mParkingSpace = 0;
          dzamm = new Markings();
          UriUj = dzamm;
          SGrXV =
              mObject;
          break;
        case ODR_OPCODE_MARKINGS_MARKING:
          if (!UriUj)
          {
            SGrXV = 0;
            break;
          }
          dzamm =
              new Marking();
          UriUj = dzamm;
          SGrXV = UriUj;
          break;
        case ODR_OPCODE_RAILROAD_SWITCH:
          dzamm = new RailroadSwitch();
          SGrXV = mRoadHeader;
          mRailroadSwitch = dzamm;
          break;
        case ODR_OPCODE_RAILROAD_MAIN_TRACK:
          dzamm = mRailroadSwitch;
          SGrXV = 0;
          if (mRailroadSwitch)
          {
            RailroadSwitch *gl9LX = reinterpret_cast<RailroadSwitch *>(mRailroadSwitch);
            gl9LX->readMainTrack();
          }
          break;
        case ODR_OPCODE_RAILROAD_SIDE_TRACK:
          dzamm = mRailroadSwitch;
          SGrXV = 0;
          if (mRailroadSwitch)
          {
            RailroadSwitch *gl9LX = reinterpret_cast<
                RailroadSwitch *>(mRailroadSwitch);
            gl9LX->readSideTrack();
          }
          break;
        case ODR_OPCODE_RAILROAD:
          mIsRailroad = true;
          break;
        case ODR_OPCODE_OBJECT_REFERENCE:
          mSignal = 0;
          mObject = new ObjectRef();
          mTunBrd = 0;
          mParkingSpace = 0;
          UriUj = 0;
          FN8fF = 0;
          NnwlR = 0;
          mDLp2 = 0;
          SGrXV = mRoadHeader;
          dzamm = mObject;
          break;
        case ODR_OPCODE_OBJECT_OUTLINE:
          dzamm = new ObjectOutline();
          SGrXV = mObject;
          O1eZ7 = dzamm;
          break;
        case ODR_OPCODE_CORNER_REFERENCE:
          if (FN8fF)
            SGrXV = FN8fF;
          else if (NnwlR)
            SGrXV =
                NnwlR;
          else
          {
            SGrXV = 0;
            break;
          }
          dzamm = new CornerReference();
          break;
        case ODR_OPCODE_BORDERS:
          dzamm = 0;
          SGrXV = 0;
          mDLp2 = true;
          break;
        case ODR_OPCODE_BORDER:
          if (!mDLp2 || !mObject)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new Border();
          NnwlR = dzamm;
          SGrXV = mObject;
          break;
        case ODR_OPCODE_REFERENCE:
          if (!mSignal)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new Reference();
          SGrXV = mSignal;
          break;
        case ODR_OPCODE_POSITION_ROAD:
          if (!mSignal)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new PositionRoad();
          SGrXV = mSignal;
          break;
        case ODR_OPCODE_POSITION_INERTIAL:
          if (!mSignal)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new PositionInertial();
          SGrXV = mSignal;
          break;
        case ODR_OPCODE_JUNCTION_LINK_PREDECESSOR:
          dzamm = new RoadLink(
              ODR_LINK_TYPE_PREDECESSOR);
          SGrXV = mJuncLink;
          break;
        case ODR_OPCODE_JUNCTION_LINK_SUCCESSOR:
          dzamm = new RoadLink(ODR_LINK_TYPE_SUCCESSOR);
          SGrXV = mJuncLink;
          break;
        case ODR_OPCODE_DATA_QUALITY:
          dzamm = new DataQuality();
          HFgoM = dzamm;
          if (SGrXV)
            dzamm->setLevel(SGrXV->getLevel() + 1);
          break;
        case ODR_OPCODE_RAW_DATA_DESC:
          if (!HFgoM)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new RawDataDesc();
          SGrXV = HFgoM;
          break;
        case ODR_OPCODE_ERROR_DESC:
          if (!HFgoM)
          {
            SGrXV = 0;
            break;
          }
          dzamm = new ErrorDesc();
          SGrXV = HFgoM;
          break;
        case ODR_OPCODE_OBJECT_MATERIAL:
        case ODR_OPCODE_ROAD_LINK:
        case ODR_OPCODE_LANE_LINK:
        case ODR_OPCODE_PLANVIEW:
        case ODR_OPCODE_ELEV_PROFILE:
        case ODR_OPCODE_LATERAL_PROFILE:
        case ODR_OPCODE_LANES:
        case ODR_OPCODE_LANES_LEFT:
        case ODR_OPCODE_LANES_CENTER:
        case ODR_OPCODE_LANES_RIGHT:
        case ODR_OPCODE_OBJECTS:
        case ODR_OPCODE_SIGNALS:
        case ODR_OPCODE_OPENDRIVE:
        case ODR_OPCODE_SURFACE:
          SGrXV = 0;
          break;
        default:
          std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x20\x75\x6e\x68\x61\x6e\x64\x6c\x65\x64\x20\x6f\x70\x63\x6f\x64\x65\x20"
                    << CvVIp << "\x20\x66\x6f\x72\x20\x74\x61\x67\x20\x3c" << HLfEO->Value() << "\x3e" << std ::endl;
          UmrFZ = true;
        }
      }
      if (dzamm)
      {
        if (mAddMode)
        {
          if (mFlags & FLAG_ADD_SIGNALS)
          {
            switch (
                CvVIp)
            {
            case ODR_OPCODE_SIGNAL_REFERENCE:
            case ODR_OPCODE_SIGNAL:
              SGrXV = mRoadHeader;
              break;
            case ODR_OPCODE_SIGNAL_DEPEND:
              break;
            case ODR_OPCODE_USER_DATA:
              if (!mSignal)
                SGrXV = 0;
              break;
            default:
              SGrXV = 0;
              break;
            }
          }
        }
        if (SGrXV)
          SGrXV->addChild(dzamm);
        dzamm->setLineNo(HLfEO->Row(), mNoModif);
        TiXmlElement *p_heP = HLfEO->ToElement();
        if (p_heP)
        {
          TiXmlAttribute *dmeHa = p_heP->FirstAttribute();
          while (dmeHa)
          {
            addAttribute(dmeHa->Name(), dmeHa->Value());
            dmeHa = dmeHa->Next();
          }
        }
        dzamm->read(this);
        clearAttributes();
        if (mVerbose)
        {
          std::cerr << "\x2e\x64\x6f\x6e\x65\x2e\x20\x54\x68\x65\x20\x66\x6f\x6c\x6c\x6f\x77\x69\x6e\x67\x20\x64\x61\x74\x61\x20\x68\x61\x73\x20\x62\x65\x65\x6e\x20\x72\x65\x61\x64\x3a"
                    << std::endl;
          dzamm->print();
        }
        if (mAddMode)
        {
          if (mFlags & FLAG_ADD_SIGNALS)
          {
            switch (
                CvVIp)
            {
            case ODR_OPCODE_ROAD_HEADER:
            {
              std::string DAlxN = (reinterpret_cast<
                                       RoadHeader *>(dzamm))
                                      ->mIdAsString;
              mRoadHeader = RoadData::getInstance()->findFirstNode(CvVIp);
              while (mRoadHeader && (DAlxN != (reinterpret_cast<RoadHeader *>(
                                                   mRoadHeader))
                                                  ->mIdAsString))
                mRoadHeader = mRoadHeader->getRight();
              if (!mRoadHeader)
                std::cerr << "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x46\x69\x6c\x65\x3a\x20\x6e\x6f\x20\x65\x78\x69\x73\x74\x69\x6e\x67\x20\x72\x6f\x61\x64\x20\x68\x65\x61\x64\x65\x72\x20\x66\x6f\x72\x20\x69\x64\x20"
                          << DAlxN << "\x20\x66\x6f\x75\x6e\x64\x21" << std::endl;
              delete dzamm;
              dzamm = 0;
            }
            break;
            case ODR_OPCODE_SIGNAL:
            case ODR_OPCODE_SIGNAL_REFERENCE:
            case ODR_OPCODE_SIGNAL_DEPEND:
              break;
            case ODR_OPCODE_USER_DATA:
              if (!mSignal)
              {
                delete dzamm;
                dzamm = 0;
              }
              break;
            default:
              delete dzamm;
              dzamm = 0;
              break;
            }
          }
        }
        if (CvVIp !=
            ODR_OPCODE_USER_DATA)
          mLastNewNode = dzamm;
      }
    }
    else
    {
      switch (rkiXc)
      {
      case TiXmlNode::
          DOCUMENT:
        if (mVerbose)
          std::cerr << "\x44\x6f\x63\x75\x6d\x65\x6e\x74" << std::endl;
        break;
      case TiXmlNode::COMMENT:
        if (mVerbose)
          std::cerr << "\x43\x6f\x6d\x6d\x65\x6e\x74\x20\x5b" << HLfEO->Value() << "\x5d" << std::endl;
        break;
      case TiXmlNode::UNKNOWN:
        std::cerr << "\x64\x65\x74\x65\x63\x74\x65\x64\x20\x75\x6e\x6b\x6e\x6f\x77\x6e\x20\x65\x6e\x74\x72\x79\x20"
                  << std::endl;
        break;
      case TiXmlNode::TEXT:
        j2ESK = HLfEO->ToText();
        if (mVerbose)
          std::
                  cerr
              << "\x54\x65\x78\x74\x20\x5b" << j2ESK->Value() << "\x5d" << std::endl;
        if (
            mLastNewNode)
          mLastNewNode->setCDATA(j2ESK->Value());
        break;
      case TiXmlNode::
          DECLARATION:
        if (mVerbose)
          std::cerr << "\x44\x65\x63\x6c\x61\x72\x61\x74\x69\x6f\x6e" << std::endl;
        break;
      }
    }
    if (YXdS5)
    {
      for (
          iaLTA = HLfEO->FirstChild(); iaLTA != 0; iaLTA = iaLTA->NextSibling())
      {
        parseFile(iaLTA,
                  UmrFZ, dzamm);
      }
    }
    if (dzamm && (dzamm == mLaneSec))
    {
      if (mAddBorderLanes)
      {
        (
            reinterpret_cast<LaneSection *>(mLaneSec))
            ->addBorderLanes(mBorderWidth,
                             ODR_LANE_TYPE_NONE, mBorderWidth, ODR_LANE_TYPE_NONE);
      }
    }
    return true;
  }
  unsigned char
  ReaderXML::getUChar(const std::string &Gqdbt) { return (unsigned char)atoi(
      getString(Gqdbt).c_str()); }
  unsigned short ReaderXML::getUShort(const std::string
                                          &Gqdbt) { return (unsigned short)atoi(getString(Gqdbt).c_str()); }
  int ReaderXML::
      getInt(const std::string &Gqdbt) { return atoi(getString(Gqdbt).c_str()); }
  unsigned int ReaderXML::getUInt(const std::string &Gqdbt) { return (unsigned int)atoi(
      getString(Gqdbt).c_str()); }
  double ReaderXML::getDouble(const std::string &Gqdbt)
  {
    return atof(getString(Gqdbt).c_str());
  }
  std::string ReaderXML::getString(const std::string &Gqdbt)
  {
    AttribMap::iterator VfDVC = mAttribMap.find(Gqdbt);
    if (VfDVC ==
        mAttribMap.end())
      return "\x75\x6e\x6b\x6e\x6f\x77\x6e";
    return VfDVC->second;
  }
  unsigned int ReaderXML::getOpcodeFromLaneType(const std::string &name)
  {
    StringValMap::iterator VfDVC = mLaneTypeMap.find(loCase(name));
    if (VfDVC ==
        mLaneTypeMap.end())
      return ODR_LANE_TYPE_NONE;
    return VfDVC->second;
  }
  unsigned int
  ReaderXML::getOpcodeFromRoadMarkType(const std::string &name)
  {
    StringValMap::
        iterator VfDVC = mRoadMarkTypeMap.find(loCase(name));
    if (VfDVC == mRoadMarkTypeMap.end())
      return ODR_ROAD_MARK_TYPE_NONE;
    return VfDVC->second;
  }
  unsigned int
  ReaderXML::getOpcodeFromRoadMarkWeight(const std::string &name)
  {
    StringValMap::
        iterator VfDVC = mRoadMarkWeightMap.find(loCase(name));
    if (VfDVC ==
        mRoadMarkWeightMap.end())
      return ODR_ROAD_MARK_WEIGHT_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::getOpcodeFromRoadMarkColor(const std::string &name)
  {
    StringValMap::iterator VfDVC = mRoadMarkColorMap.find(loCase(name));
    if (VfDVC ==
        mRoadMarkColorMap.end())
      return ODR_ROAD_MARK_COLOR_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::getOpcodeFromElement(const std::string &name)
  {
    StringValMap::iterator VfDVC = mOpcodeMap.find(loCase(name));
    if (VfDVC == mOpcodeMap.end())
      return ODR_OPCODE_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromLaneAccess(const std::string &name)
  {
    StringValMap::iterator VfDVC =
        mLaneAccessMap.find(loCase(name));
    if (VfDVC == mLaneAccessMap.end())
      return ODR_LANE_ACCESS_RESTRICT_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromTunnelType(const std::string &name)
  {
    StringValMap::iterator VfDVC =
        mTunnelTypeMap.find(loCase(name));
    if (VfDVC == mTunnelTypeMap.end())
      return ODR_TUNNEL_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromBridgeType(const std::string &name)
  {
    StringValMap::iterator VfDVC =
        mBridgeTypeMap.find(loCase(name));
    if (VfDVC == mBridgeTypeMap.end())
      return ODR_BRIDGE_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromCountry(const std::string &name)
  {
    StringValMap::iterator VfDVC =
        mCountryMap.find(loCase(name));
    if (VfDVC == mCountryMap.end())
      return ODR_COUNTRY_OPENDRIVE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromRoadType(const std::string &name)
  {
    StringValMap::iterator VfDVC =
        mRoadTypeMap.find(loCase(name));
    if (VfDVC == mRoadTypeMap.end())
      return ODR_ROAD_TYPE_NONE;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      getOpcodeFromUnit(const std::string &name)
  {
    StringValMap::iterator VfDVC = mUnitMap.find(loCase(name));
    if (VfDVC == mUnitMap.end())
      return ODR_UNIT_DEFAULT;
    return VfDVC
        ->second;
  }
  unsigned int ReaderXML::getOpcodeFromRoadMarkRule(const std::string &
                                                        name)
  {
    StringValMap::iterator VfDVC = mRoadMarkRuleMap.find(loCase(name));
    if (VfDVC == mRoadMarkRuleMap.end())
      return ODR_ROAD_MARK_RULE_DEFAULT;
    return VfDVC->second;
  }
  unsigned int ReaderXML::CgGZS(const std::string &name)
  {
    StringValMap::iterator
        VfDVC = QUzzb.find(loCase(name));
    if (VfDVC == QUzzb.end())
      return ODR_OUTLINE_FILL_TYPE_DEFAULT;
    return VfDVC->second;
  }
  unsigned int ReaderXML::
      zeDa8(const std::string &name)
  {
    StringValMap::iterator VfDVC = hYi15.find(loCase(
        name));
    if (VfDVC == hYi15.end())
      return ODR_OBJECT_TYPE_NONE;
    return VfDVC->second;
  }
  bool ReaderXML::hasAttribute(const std::string &Gqdbt)
  {
    AttribMap::iterator VfDVC =
        mAttribMap.find(Gqdbt);
    return (VfDVC != mAttribMap.end());
  }
  void ReaderXML::SrdMD(
      bool F744N) { mResolveRepeatedObjects = F744N; }
  void ReaderXML::Fa4eH(const float &
                            value) { mContObjTesselation = fabs(value); }
  bool ReaderXML::parseGenericNode(void *
                                       GaK8N,
                                   Node &XFL9o)
  {
    TiXmlElement *fbjf2 = static_cast<TiXmlElement *>(GaK8N);
    if (!fbjf2)
      return false;
    if (mVerbose)
    {
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x47\x65\x6e\x65\x72\x69\x63\x4e\x6f\x64\x65\x3a\x20\x61\x64\x64\x72\x65\x73\x73\x20\x6f\x66\x20\x70\x61\x72\x65\x6e\x74\x20\x3d\x20\x25\x70"
              "\n",
              &XFL9o);
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x47\x65\x6e\x65\x72\x69\x63\x4e\x6f\x64\x65\x3a\x20\x74\x79\x70\x65\x20\x6f\x66\x20\x70\x61\x72\x65\x6e\x74\x20\x3d\x20\x25\x64"
              "\n",
              XFL9o.getOpcode());
    }
    for (TiXmlElement *BnVZ6 = fbjf2->FirstChildElement(); BnVZ6;
         BnVZ6 = BnVZ6->NextSiblingElement())
    {
      if (mVerbose)
        fprintf(stderr,
                "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x70\x61\x72\x73\x65\x47\x65\x6e\x65\x72\x69\x63\x4e\x6f\x64\x65\x3a\x20\x74\x79\x70\x65\x20\x6f\x66\x20\x70\x61\x72\x65\x6e\x74\x32\x20\x3d\x20\x25\x64"
                "\n",
                XFL9o.getOpcode());
      GenericNode *ga9wG = new GenericNode(BnVZ6->Value());
      XFL9o.addChild(ga9wG);
      ga9wG->setLevel(XFL9o.getLevel() + 1);
      ga9wG->setLineNo(BnVZ6->Row(), mNoModif);
      clearAttributes();
      TiXmlAttribute *dmeHa = BnVZ6->FirstAttribute();
      while (dmeHa)
      {
        addAttribute(dmeHa->Name(), dmeHa->Value());
        dmeHa = dmeHa->Next();
      }
      ga9wG->read(this);
      clearAttributes();
      parseGenericNode(BnVZ6, *ga9wG);
    }
    return true;
  }
  void
  ReaderXML::initOpcodes()
  {
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x68\x65\x61\x64\x65\x72", ODR_OPCODE_HEADER));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>("\x72\x6f\x61\x64", ODR_OPCODE_ROAD_HEADER));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x70\x72\x65\x64\x65\x63\x65\x73\x73\x6f\x72", ODR_OPCODE_PREDECESSOR));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x75\x63\x63\x65\x73\x73\x6f\x72", ODR_OPCODE_SUCCESSOR));
    mOpcodeMap.insert(
        std::pair<std::string, unsigned int>("\x74\x79\x70\x65", ODR_OPCODE_ROAD_TYPE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x67\x65\x6f\x6d\x65\x74\x72\x79", ODR_OPCODE_GEO_HEADER));
    mOpcodeMap.insert(std ::pair<std::string, unsigned int>("\x6c\x69\x6e\x65", ODR_OPCODE_GEO_LINE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x73\x70\x69\x72\x61\x6c", ODR_OPCODE_GEO_SPIRAL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x61\x72\x63", ODR_OPCODE_GEO_ARC));
    mOpcodeMap.insert(std::pair<std::string,
                                unsigned int>("\x65\x6c\x65\x76\x61\x74\x69\x6f\x6e", ODR_OPCODE_ELEVATION));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x6c\x61\x6e\x65\x73\x65\x63\x74\x69\x6f\x6e", ODR_OPCODE_LANE_SECTION));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6c\x61\x6e\x65",
                                                           ODR_OPCODE_LANE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x6c\x69\x6e\x6b", ODR_OPCODE_LANE_LINK));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>("\x77\x69\x64\x74\x68", ODR_OPCODE_LANE_WIDTH));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x72\x6f\x61\x64\x6d\x61\x72\x6b",
                                                           ODR_OPCODE_LANE_ROAD_MARK));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6d\x61\x74\x65\x72\x69\x61\x6c", ODR_OPCODE_LANE_MATERIAL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x76\x69\x73\x69\x62\x69\x6c\x69\x74\x79", ODR_OPCODE_LANE_VISIBILITY));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x73\x69\x67\x6e\x61\x6c", ODR_OPCODE_SIGNAL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x76\x61\x6c\x69\x64\x69\x74\x79", ODR_OPCODE_LANE_VALIDITY));
    mOpcodeMap.insert(
        std::pair<std::string, unsigned int>(
            "\x6c\x61\x6e\x65\x56\x61\x6c\x69\x64\x69\x74\x79", ODR_OPCODE_LANE_VALIDITY));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x64\x65\x70\x65\x6e\x64\x65\x6e\x63\x79", ODR_OPCODE_SIGNAL_DEPEND));
    mOpcodeMap
        .insert(std::pair<std::string, unsigned int>(
            "\x63\x6f\x6e\x74\x72\x6f\x6c\x6c\x65\x72", ODR_OPCODE_CONTROLLER));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x63\x6f\x6e\x74\x72\x6f\x6c",
                                                           ODR_OPCODE_CONTROL_ENTRY));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6a\x75\x6e\x63\x74\x69\x6f\x6e", ODR_OPCODE_JUNCTION_HEADER));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x63\x6f\x6e\x6e\x65\x63\x74\x69\x6f\x6e", ODR_OPCODE_JUNCTION_LINK));
    mOpcodeMap
        .insert(std::pair<std::string, unsigned int>("\x6c\x61\x6e\x65\x6c\x69\x6e\x6b",
                                                     ODR_OPCODE_JUNCTION_LANE_LINK));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x70\x72\x69\x6f\x72\x69\x74\x79", ODR_OPCODE_JUNCTION_PRIORITY));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6f\x62\x6a\x65\x63\x74", ODR_OPCODE_OBJECT));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x75\x73\x65\x72\x64\x61\x74\x61", ODR_OPCODE_USER_DATA));
    mOpcodeMap.insert(std ::pair<std::string, unsigned int>("\x63\x72\x6f\x73\x73\x66\x61\x6c\x6c",
                                                            ODR_OPCODE_CROSSFALL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x70\x6c\x61\x6e\x76\x69\x65\x77", ODR_OPCODE_PLANVIEW));
    mOpcodeMap.insert(std::
                          pair<std::string, unsigned int>(
                              "\x65\x6c\x65\x76\x61\x74\x69\x6f\x6e\x70\x72\x6f\x66\x69\x6c\x65",
                              ODR_OPCODE_ELEV_PROFILE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x6c\x61\x74\x65\x72\x61\x6c\x70\x72\x6f\x66\x69\x6c\x65",
        ODR_OPCODE_LATERAL_PROFILE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6c\x61\x6e\x65\x73", ODR_OPCODE_LANES));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>("\x6c\x65\x66\x74", ODR_OPCODE_LANES_LEFT));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x63\x65\x6e\x74\x65\x72",
                                                           ODR_OPCODE_LANES_CENTER));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x72\x69\x67\x68\x74", ODR_OPCODE_LANES_RIGHT));
    mOpcodeMap.insert(std::pair<std ::string, unsigned int>("\x72\x6f\x61\x64\x6d\x61\x72\x6b",
                                                            ODR_OPCODE_LANE_ROAD_MARK));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6f\x62\x6a\x65\x63\x74\x73", ODR_OPCODE_OBJECTS));
    mOpcodeMap.insert(std::
                          pair<std::string, unsigned int>("\x73\x69\x67\x6e\x61\x6c\x73", ODR_OPCODE_SIGNALS));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x6f\x70\x65\x6e\x64\x72\x69\x76\x65", ODR_OPCODE_OPENDRIVE));
    mOpcodeMap.insert(
        std::pair<std::string, unsigned int>(
            "\x73\x75\x70\x65\x72\x65\x6c\x65\x76\x61\x74\x69\x6f\x6e",
            ODR_OPCODE_SUPERELEVATION));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x70\x6f\x6c\x79\x33", ODR_OPCODE_GEO_POLY));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>("\x73\x70\x65\x65\x64", ODR_OPCODE_LANE_SPEED));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x61\x63\x63\x65\x73\x73",
                                                           ODR_OPCODE_LANE_ACCESS));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x68\x65\x69\x67\x68\x74", ODR_OPCODE_LANE_HEIGHT));
    mOpcodeMap.insert(std::pair<
                      std::string, unsigned int>("\x72\x75\x6c\x65", ODR_OPCODE_LANE_RULE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x63\x6f\x72\x6e\x65\x72\x69\x6e\x65\x72\x74\x69\x61\x6c",
        ODR_OPCODE_CORNER_INERTIAL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x63\x6f\x72\x6e\x65\x72\x72\x6f\x61\x64", ODR_OPCODE_CORNER_ROAD));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x63\x6f\x72\x6e\x65\x72\x72\x65\x6c\x61\x74\x69\x76\x65",
        ODR_OPCODE_CORNER_RELATIVE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x63\x6f\x72\x6e\x65\x72\x6c\x6f\x63\x61\x6c", ODR_OPCODE_CORNER_LOCAL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x74\x75\x6e\x6e\x65\x6c", ODR_OPCODE_TUNNEL));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x62\x72\x69\x64\x67\x65", ODR_OPCODE_BRIDGE));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>(
        "\x73\x69\x67\x6e\x61\x6c\x72\x65\x66\x65\x72\x65\x6e\x63\x65",
        ODR_OPCODE_SIGNAL_REFERENCE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6f\x75\x74\x6c\x69\x6e\x65", ODR_OPCODE_OBJECT_OUTLINE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x6c\x61\x6e\x65\x6f\x66\x66\x73\x65\x74", ODR_OPCODE_LANE_OFFSET));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x73\x75\x72\x66\x61\x63\x65",
                                                           ODR_OPCODE_SURFACE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x43\x52\x47", ODR_OPCODE_SURFACE_CRG));
    mOpcodeMap.insert(std::pair<std::string,
                                unsigned int>("\x72\x65\x70\x65\x61\x74", ODR_OPCODE_REPEAT));
    mOpcodeMap.insert(
        std::pair<std::string, unsigned int>("\x70\x61\x72\x61\x6d\x70\x6f\x6c\x79\x33",
                                             ODR_OPCODE_GEO_PARAM_POLY));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x62\x6f\x72\x64\x65\x72", ODR_OPCODE_LANE_BORDER));
    mOpcodeMap.insert(std::
                          pair<std::string, unsigned int>(
                              "\x67\x65\x6f\x72\x65\x66\x65\x72\x65\x6e\x63\x65", ODR_OPCODE_GEO_REFERENCE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6f\x66\x66\x73\x65\x74", ODR_OPCODE_OFFSET));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x68\x61\x70\x65", ODR_OPCODE_LATERAL_SHAPE));
    mOpcodeMap.insert(std::pair<
                      std::string, unsigned int>("\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x67\x72\x6f\x75\x70", ODR_OPCODE_JUNCTION_GROUP));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x72\x65\x66\x65\x72\x65\x6e\x63\x65",
                                                           ODR_OPCODE_JUNCTION_REFERENCE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x70\x61\x72\x6b\x69\x6e\x67\x73\x70\x61\x63\x65",
                                                           ODR_OPCODE_PARKING_SPACE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6e\x65\x69\x67\x68\x62\x6f\x72", ODR_OPCODE_NEIGHBOR));
    mOpcodeMap.insert(std ::pair<std::string, unsigned int>("\x72\x61\x69\x6c\x72\x6f\x61\x64",
                                                            ODR_OPCODE_RAILROAD));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x77\x69\x74\x63\x68", ODR_OPCODE_SWITCH));
    mOpcodeMap.insert(std::pair<std::
                                    string,
                                unsigned int>("\x6d\x61\x69\x6e\x54\x72\x61\x63\x6b",
                                              ODR_OPCODE_RAILROAD_MAIN_TRACK));
    mOpcodeMap.insert(std::pair<std::string,
                                unsigned int>("\x73\x69\x64\x65\x54\x72\x61\x63\x6b",
                                              ODR_OPCODE_RAILROAD_SIDE_TRACK));
    mOpcodeMap.insert(std::pair<std::string,
                                unsigned int>("\x6f\x62\x6a\x65\x63\x74\x72\x65\x66\x65\x72\x65\x6e\x63\x65",
                                              ODR_OPCODE_OBJECT_REFERENCE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x73\x77\x61\x79", ODR_OPCODE_ROAD_MARK_SWAY));
    mOpcodeMap.insert(std::pair<
                      std::string, unsigned int>("\x65\x78\x70\x6c\x69\x63\x69\x74",
                                                 ODR_OPCODE_ROAD_MARK_EXPLICIT));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6d\x61\x72\x6b\x69\x6e\x67\x73", ODR_OPCODE_MARKINGS));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x6d\x61\x72\x6b\x69\x6e\x67",
                                                           ODR_OPCODE_MARKINGS_MARKING));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x63\x6f\x72\x6e\x65\x72\x52\x65\x66\x65\x72\x65\x6e\x63\x65",
                                                           ODR_OPCODE_CORNER_REFERENCE));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>("\x62\x6f\x72\x64\x65\x72\x73", ODR_OPCODE_BORDERS));
    mOpcodeMap.insert(std::
                          pair<std::string, unsigned int>("\x62\x6f\x72\x64\x65\x72", ODR_OPCODE_BORDER));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x72\x65\x66\x65\x72\x65\x6e\x63\x65", ODR_OPCODE_REFERENCE));
    mOpcodeMap.insert(
        std::pair<std::string, unsigned int>(
            "\x70\x6f\x73\x69\x74\x69\x6f\x6e\x52\x6f\x61\x64", ODR_OPCODE_POSITION_ROAD));
    mOpcodeMap.insert(std::pair<std::string, unsigned int>(
        "\x70\x6f\x73\x69\x74\x69\x6f\x6e\x49\x6e\x65\x72\x74\x69\x61\x6c",
        ODR_OPCODE_POSITION_INERTIAL));
    mLaneTypeMap.insert(std::pair<std::string,
                                  unsigned int>("\x6e\x6f\x6e\x65", ODR_LANE_TYPE_NONE));
    mLaneTypeMap.insert(std::
                            pair<std::string, unsigned int>("\x64\x72\x69\x76\x69\x6e\x67",
                                                            ODR_LANE_TYPE_DRIVING));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x74\x6f\x70", ODR_LANE_TYPE_STOP));
    mLaneTypeMap.insert(std::pair<std::
                                      string,
                                  unsigned int>("\x73\x68\x6f\x75\x6c\x64\x65\x72", ODR_LANE_TYPE_SHOULDER));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x62\x69\x6b\x69\x6e\x67", ODR_LANE_TYPE_BIKING));
    mLaneTypeMap.insert(std::pair<
                        std::string, unsigned int>("\x73\x69\x64\x65\x77\x61\x6c\x6b",
                                                   ODR_LANE_TYPE_SIDEWALK));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x62\x6f\x72\x64\x65\x72", ODR_LANE_TYPE_BORDER));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x72\x65\x73\x74\x72\x69\x63\x74\x65\x64",
                                                             ODR_LANE_TYPE_RESTRICTED));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x70\x61\x72\x6b\x69\x6e\x67", ODR_LANE_TYPE_PARKING));
    mLaneTypeMap.insert(
        std::pair<std::string, unsigned int>("\x6d\x77\x79\x45\x6e\x74\x72\x79",
                                             ODR_LANE_TYPE_MWY_ENTRY));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x6d\x77\x79\x45\x78\x69\x74", ODR_LANE_TYPE_MWY_EXIT));
    mLaneTypeMap.insert(
        std::pair<std::string, unsigned int>("\x65\x6e\x74\x72\x79", ODR_LANE_TYPE_ENTRY));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x65\x78\x69\x74",
                                                             ODR_LANE_TYPE_EXIT));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x70\x65\x63\x69\x61\x6c\x31", ODR_LANE_TYPE_SPECIAL1));
    mLaneTypeMap.insert(
        std::pair<std::string, unsigned int>("\x73\x70\x65\x63\x69\x61\x6c\x32",
                                             ODR_LANE_TYPE_SPECIAL2));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x73\x70\x65\x63\x69\x61\x6c\x33", ODR_LANE_TYPE_SPECIAL3));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x73\x70\x65\x63\x69\x61\x6c\x34",
                                                             ODR_LANE_TYPE_SPECIAL4));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x72\x6f\x61\x64\x57\x6f\x72\x6b\x73", ODR_LANE_TYPE_DRIVING_ROADWORKS));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x72\x61\x69\x6c",
                                                             ODR_LANE_TYPE_RAIL));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x74\x72\x61\x6d", ODR_LANE_TYPE_TRAM));
    mLaneTypeMap.insert(std::pair<std::
                                      string,
                                  unsigned int>("\x62\x69\x64\x69\x72\x65\x63\x74\x69\x6f\x6e\x61\x6c",
                                                ODR_LANE_TYPE_BIDIRECTIONAL));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x6d\x65\x64\x69\x61\x6e", ODR_LANE_TYPE_MEDIAN));
    mLaneTypeMap.insert(std ::pair<std::string, unsigned int>("\x6f\x6e\x52\x61\x6d\x70",
                                                              ODR_LANE_TYPE_ON_RAMP));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x6f\x66\x66\x52\x61\x6d\x70", ODR_LANE_TYPE_OFF_RAMP));
    mLaneTypeMap.insert(std ::pair<std::string, unsigned int>(
        "\x63\x6f\x6e\x6e\x65\x63\x74\x69\x6e\x67\x52\x61\x6d\x70",
        ODR_LANE_TYPE_CONNECTING_RAMP));
    mLaneTypeMap.insert(std::pair<std::string,
                                  unsigned int>("\x62\x75\x73", ODR_LANE_TYPE_BUS));
    mLaneTypeMap.insert(std::pair<
                        std::string, unsigned int>("\x74\x61\x78\x69", ODR_LANE_TYPE_TAXI));
    mLaneTypeMap.insert(std::pair<std::string, unsigned int>("\x48\x4f\x56", ODR_LANE_TYPE_HOV));
    mRoadMarkTypeMap.insert(std::pair<std::string, unsigned int>("\x6e\x6f\x6e\x65",
                                                                 ODR_ROAD_MARK_TYPE_NONE));
    mRoadMarkTypeMap.insert(std::pair<std::string, unsigned int>("\x73\x6f\x6c\x69\x64", ODR_ROAD_MARK_TYPE_SOLID));
    mRoadMarkTypeMap.insert(
        std::pair<std::string, unsigned int>("\x62\x72\x6f\x6b\x65\x6e",
                                             ODR_ROAD_MARK_TYPE_BROKEN));
    mRoadMarkTypeMap.insert(std::pair<std::string,
                                      unsigned int>("\x73\x6f\x6c\x69\x64\x20\x73\x6f\x6c\x69\x64",
                                                    ODR_ROAD_MARK_TYPE_SOLID_SOLID));
    mRoadMarkTypeMap.insert(std::pair<std::string,
                                      unsigned int>("\x73\x6f\x6c\x69\x64\x20\x62\x72\x6f\x6b\x65\x6e",
                                                    ODR_ROAD_MARK_TYPE_SOLID_BROKEN));
    mRoadMarkTypeMap.insert(std::pair<std::string,
                                      unsigned int>("\x62\x72\x6f\x6b\x65\x6e\x20\x73\x6f\x6c\x69\x64",
                                                    ODR_ROAD_MARK_TYPE_BROKEN_SOLID));
    mRoadMarkTypeMap.insert(std::pair<std::string,
                                      unsigned int>("\x62\x72\x6f\x6b\x65\x6e\x20\x62\x72\x6f\x6b\x65\x6e",
                                                    ODR_ROAD_MARK_TYPE_BROKEN_BROKEN));
    mRoadMarkTypeMap.insert(std::pair<std::string, unsigned int>("\x63\x75\x72\x62", ODR_ROAD_MARK_TYPE_CURB));
    mRoadMarkTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x62\x6f\x74\x74\x73\x20\x64\x6f\x74\x73", ODR_ROAD_MARK_TYPE_BOTTS_DOTS));
    mRoadMarkTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x63\x75\x73\x74\x6f\x6d", ODR_ROAD_MARK_TYPE_CUSTOM));
    mRoadMarkTypeMap.insert(
        std::pair<std::string, unsigned int>("\x65\x64\x67\x65", ODR_ROAD_MARK_TYPE_EDGE));
    mRoadMarkWeightMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x74\x61\x6e\x64\x61\x72\x64", ODR_ROAD_MARK_WEIGHT_STANDARD));
    mRoadMarkWeightMap.insert(std::pair<std::string, unsigned int>("\x62\x6f\x6c\x64", ODR_ROAD_MARK_WEIGHT_BOLD));
    mRoadMarkColorMap.insert(std::pair<std::string,
                                       unsigned int>("\x73\x74\x61\x6e\x64\x61\x72\x64", ODR_ROAD_MARK_COLOR_STANDARD));
    mRoadMarkColorMap.insert(std::pair<std::string, unsigned int>(
        "\x79\x65\x6c\x6c\x6f\x77", ODR_ROAD_MARK_COLOR_YELLOW));
    mRoadMarkColorMap.insert(std::pair<std::string, unsigned int>("\x72\x65\x64", ODR_ROAD_MARK_COLOR_RED));
    mRoadMarkColorMap.insert(std::pair<std::string, unsigned int>(
        "\x77\x68\x69\x74\x65", ODR_ROAD_MARK_COLOR_WHITE));
    mRoadMarkColorMap.insert(std ::pair<std::string, unsigned int>("\x67\x72\x65\x65\x6e",
                                                                   ODR_ROAD_MARK_COLOR_GREEN));
    mRoadMarkColorMap.insert(std::pair<std::string,
                                       unsigned int>("\x62\x6c\x75\x65", ODR_ROAD_MARK_COLOR_BLUE));
    mRoadMarkColorMap.insert(std::pair<std::string, unsigned int>("\x6f\x72\x61\x6e\x67\x65",
                                                                  ODR_ROAD_MARK_COLOR_ORANGE));
    mLaneAccessMap.insert(std::pair<std::string,
                                    unsigned int>("\x73\x69\x6d\x75\x6c\x61\x74\x6f\x72",
                                                  ODR_LANE_ACCESS_RESTRICT_SIMULATOR));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>(
        "\x61\x75\x74\x6f\x6e\x6f\x6d\x6f\x75\x73\x20\x74\x72\x61\x66\x66\x69\x63",
        ODR_LANE_ACCESS_RESTRICT_AUTONOMOUS));
    mLaneAccessMap.insert(std::pair<std::
                                        string,
                                    unsigned int>("\x70\x65\x64\x65\x73\x74\x72\x69\x61\x6e",
                                                  ODR_LANE_ACCESS_RESTRICT_PEDESTRIAN));
    mLaneAccessMap.insert(std::pair<std::
                                        string,
                                    unsigned int>("\x70\x61\x73\x73\x65\x6e\x67\x65\x72\x43\x61\x72",
                                                  ODR_LANE_ACCESS_RESTRICT_PASSENGER_CAR));
    mLaneAccessMap.insert(std::pair<std::
                                        string,
                                    unsigned int>("\x62\x75\x73", ODR_LANE_ACCESS_RESTRICT_BUS));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>(
        "\x64\x65\x6c\x69\x76\x65\x72\x79", ODR_LANE_ACCESS_RESTRICT_DELIVERY));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>(
        "\x65\x6d\x65\x72\x67\x65\x6e\x63\x79", ODR_LANE_ACCESS_RESTRICT_EMERGENCY));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>("\x74\x61\x78\x69",
                                                               ODR_LANE_ACCESS_RESTRICT_TAXI));
    mLaneAccessMap.insert(std::pair<std::string,
                                    unsigned int>("\x74\x68\x72\x6f\x75\x67\x68\x54\x72\x61\x66\x66\x69\x63",
                                                  ODR_LANE_ACCESS_RESTRICT_THROUGH_TRAFFIC));
    mLaneAccessMap.insert(std::pair<std::
                                        string,
                                    unsigned int>("\x74\x72\x75\x63\x6b", ODR_LANE_ACCESS_RESTRICT_TRUCK));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>(
        "\x62\x69\x63\x79\x63\x6c\x65", ODR_LANE_ACCESS_RESTRICT_BICYCLE));
    mLaneAccessMap
        .insert(std::pair<std::string, unsigned int>(
            "\x6d\x6f\x74\x6f\x72\x63\x79\x63\x6c\x65", ODR_LANE_ACCESS_RESTRICT_MOTORCYCLE));
    mLaneAccessMap.insert(std::pair<std::string, unsigned int>(
        "\x61\x6c\x6c\x6f\x77", ODR_LANE_ACCESS_RESTRICT_ALLOW));
    mLaneAccessMap.insert(
        std::pair<std::string, unsigned int>("\x64\x65\x6e\x79",
                                             ODR_LANE_ACCESS_RESTRICT_DENY));
    mTunnelTypeMap.insert(std::pair<std::string,
                                    unsigned int>("\x73\x74\x61\x6e\x64\x61\x72\x64", ODR_TUNNEL_STANDARD));
    mTunnelTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x75\x6e\x64\x65\x72\x70\x61\x73\x73", ODR_TUNNEL_UNDERPASS));
    mBridgeTypeMap.insert(std::pair<std::string, unsigned int>("\x63\x6f\x6e\x63\x72\x65\x74\x65",
                                                               ODR_BRIDGE_CONCRETE));
    mBridgeTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x73\x74\x65\x65\x6c", ODR_BRIDGE_STEEL));
    mBridgeTypeMap.insert(std::pair<std::
                                        string,
                                    unsigned int>("\x62\x72\x69\x63\x6b", ODR_BRIDGE_BRICK));
    mCountryMap.insert(std::pair<std::string, unsigned int>(
        "\x4f\x70\x65\x6e\x44\x52\x49\x56\x45", ODR_COUNTRY_OPENDRIVE));
    mCountryMap.insert(std::pair<std::string, unsigned int>("\x55\x53\x41", ODR_COUNTRY_USA));
    mCountryMap.insert(std::pair<std::string, unsigned int>(
        "\x46\x72\x61\x6e\x63\x65", ODR_COUNTRY_FRANCE));
    mCountryMap.insert(std::pair<std ::string, unsigned int>("\x47\x65\x72\x6d\x61\x6e\x79", ODR_COUNTRY_GERMANY));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x75\x6e\x6b\x6e\x6f\x77\x6e", ODR_ROAD_TYPE_NONE));
    mRoadTypeMap.insert(std::
                            pair<std::string, unsigned int>("\x72\x75\x72\x61\x6c", ODR_ROAD_TYPE_RURAL));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x6d\x6f\x74\x6f\x72\x77\x61\x79", ODR_ROAD_TYPE_MOTORWAY));
    mRoadTypeMap.insert(
        std::pair<std::string, unsigned int>("\x74\x6f\x77\x6e", ODR_ROAD_TYPE_TOWN));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x6c\x6f\x77\x53\x70\x65\x65\x64", ODR_ROAD_TYPE_LOW_SPEED));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>("\x70\x65\x64\x65\x73\x74\x72\x69\x61\x6e",
                                                             ODR_ROAD_TYPE_PEDESTRIAN));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>("\x62\x69\x63\x79\x63\x6c\x65", ODR_ROAD_TYPE_BICYCLE));
    mRoadTypeMap.insert(
        std::pair<std::string, unsigned int>(
            "\x74\x6f\x77\x6e\x45\x78\x70\x72\x65\x73\x73\x77\x61\x79",
            ODR_ROAD_TYPE_TOWN_EXPRESSWAY));
    mRoadTypeMap.insert(std::pair<std::string,
                                  unsigned int>("\x74\x6f\x77\x6e\x43\x6f\x6c\x6c\x65\x63\x74\x6f\x72",
                                                ODR_ROAD_TYPE_TOWN_COLLECTOR));
    mRoadTypeMap.insert(std::pair<std::string,
                                  unsigned int>("\x74\x6f\x77\x6e\x41\x72\x74\x65\x72\x69\x61\x6c",
                                                ODR_ROAD_TYPE_TOWN_ARTERIAL));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>("\x74\x6f\x77\x6e\x50\x72\x69\x76\x61\x74\x65", ODR_ROAD_TYPE_TOWN_PRIVATE));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x74\x6f\x77\x6e\x4c\x6f\x63\x61\x6c", ODR_ROAD_TYPE_TOWN_LOCAL));
    mRoadTypeMap.insert(std::pair<std::string, unsigned int>(
        "\x74\x6f\x77\x6e\x50\x6c\x61\x79\x53\x74\x72\x65\x65\x74",
        ODR_ROAD_TYPE_TOWN_PLAYSTREET));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x75\x6e\x6b\x6e\x6f\x77\x6e", ODR_UNIT_DEFAULT));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x6d", ODR_UNIT_DIST_METER));
    mUnitMap.insert(std::
                        pair<std::string, unsigned int>("\x6b\x6d", ODR_UNIT_DIST_KILOMETER));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x66\x74", ODR_UNIT_DIST_FEET));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x6d\x69\x6c\x65",
                                                         ODR_UNIT_DIST_LANDMILE));
    mUnitMap.insert(std::pair<std::string, unsigned int>(
        "\x6d\x2f\x73", ODR_UNIT_SPEED_MPS));
    mUnitMap.insert(std::pair<std::string,
                              unsigned int>("\x6b\x6d\x2f\x68", ODR_UNIT_SPEED_KMH));
    mUnitMap.insert(std::pair<
                    std::string, unsigned int>("\x6d\x70\x68", ODR_UNIT_SPEED_MPH));
    mUnitMap.insert(
        std::pair<std::string, unsigned int>("\x6b\x67", ODR_UNIT_MASS_KG));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x74", ODR_UNIT_MASS_TON));
    mUnitMap.insert(std::pair<std::string, unsigned int>("\x25", ODR_UNIT_SLOPE_PERCENT));
    mRoadMarkRuleMap.insert(std::pair<std::string, unsigned int>(
        "\x6e\x6f\x20\x70\x61\x73\x73\x69\x6e\x67", ODR_ROAD_MARK_RULE_NO_PASSING));
    mRoadMarkRuleMap.insert(std::pair<std::string, unsigned int>(
        "\x63\x61\x75\x74\x69\x6f\x6e", ODR_ROAD_MARK_RULE_CAUTION));
    mRoadMarkRuleMap.insert(std::pair<std::string, unsigned int>("\x6e\x6f\x6e\x65",
                                                                 ODR_ROAD_MARK_RULE_NONE));
    QUzzb.insert(std::pair<std::string, unsigned int>(
        "\x75\x6e\x6b\x6e\x6f\x77\x6e", ODR_OUTLINE_FILL_TYPE_DEFAULT));
    QUzzb.insert(std ::pair<std::string, unsigned int>("\x67\x72\x61\x73\x73",
                                                       ODR_OUTLINE_FILL_TYPE_GRASS));
    QUzzb.insert(std::pair<std::string, unsigned int>(
        "\x63\x6f\x6e\x63\x72\x65\x74\x65", ODR_OUTLINE_FILL_TYPE_CONCRETE));
    QUzzb.insert(std::pair<std::string, unsigned int>("\x63\x6f\x62\x62\x6c\x65",
                                                      ODR_OUTLINE_FILL_TYPE_COBBLE));
    QUzzb.insert(std::pair<std::string, unsigned int>(
        "\x61\x73\x70\x68\x61\x6c\x74", ODR_OUTLINE_FILL_TYPE_ASPHALT));
    QUzzb.insert(std ::pair<std::string, unsigned int>("\x70\x61\x76\x65\x6d\x65\x6e\x74",
                                                       ODR_OUTLINE_FILL_TYPE_PAVEMENT));
    QUzzb.insert(std::pair<std::string, unsigned int>("\x67\x72\x61\x76\x65\x6c", ODR_OUTLINE_FILL_TYPE_GRAVEL));
    QUzzb.insert(std::
                     pair<std::string, unsigned int>("\x73\x6f\x69\x6c", ODR_OUTLINE_FILL_TYPE_SOIL));
    hYi15.insert(std::pair<std::string, unsigned int>("\x75\x6e\x6b\x6e\x6f\x77\x6e",
                                                      ODR_OBJECT_TYPE_NONE));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x6e\x6f\x6e\x65", ODR_OBJECT_TYPE_NONE));
    hYi15.insert(std::pair<std::string,
                           unsigned int>("\x6f\x62\x73\x74\x61\x63\x6c\x65", ODR_OBJECT_TYPE_OBSTACLE));
    hYi15.insert(std::pair<std::string, unsigned int>("\x77\x69\x6e\x64",
                                                      ODR_OBJECT_TYPE_WIND));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x70\x6f\x6c\x65", ODR_OBJECT_TYPE_POLE));
    hYi15.insert(std::pair<std::string,
                           unsigned int>("\x74\x72\x65\x65", ODR_OBJECT_TYPE_TREE));
    hYi15.insert(std::pair<
                 std::string, unsigned int>("\x76\x65\x67\x65\x74\x61\x74\x69\x6f\x6e",
                                            ODR_OBJECT_TYPE_VEGETATION));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x62\x61\x72\x72\x69\x65\x72", ODR_OBJECT_TYPE_BARRIER));
    hYi15.insert(std::pair<
                 std::string, unsigned int>("\x62\x75\x69\x6c\x64\x69\x6e\x67",
                                            ODR_OBJECT_TYPE_BUILDING));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x70\x61\x72\x6b\x69\x6e\x67\x53\x70\x61\x63\x65", ODR_OBJECT_TYPE_PARKING_SPACE));
    hYi15.insert(std::pair<std::string, unsigned int>("\x70\x61\x74\x63\x68",
                                                      ODR_OBJECT_TYPE_PATCH));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x72\x61\x69\x6c\x69\x6e\x67", ODR_OBJECT_TYPE_RAILING));
    hYi15.insert(std::pair<
                 std::string, unsigned int>("\x74\x72\x61\x66\x66\x69\x63\x49\x73\x6c\x61\x6e\x64", ODR_OBJECT_TYPE_TRAFFIC_ISLAND));
    hYi15.insert(std::pair<std::string, unsigned int>("\x63\x72\x6f\x73\x73\x77\x61\x6c\x6b", ODR_OBJECT_TYPE_CROSSWALK));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x73\x74\x72\x65\x65\x74\x4c\x61\x6d\x70", ODR_OBJECT_TYPE_STREETLAMP));
    hYi15.insert(std::pair<std::string, unsigned int>("\x67\x61\x6e\x74\x72\x79",
                                                      ODR_OBJECT_TYPE_GANTRY));
    hYi15.insert(std::pair<std::string, unsigned int>(
        "\x73\x6f\x75\x6e\x64\x42\x61\x72\x72\x72\x69\x65\x72",
        ODR_OBJECT_TYPE_SOUND_BARRIER));
    prepareMap(mOpcodeMap);
    prepareMap(mLaneTypeMap);
    prepareMap(mRoadMarkTypeMap);
    prepareMap(mRoadMarkWeightMap);
    prepareMap(
        mLaneAccessMap);
    prepareMap(mTunnelTypeMap);
    prepareMap(mBridgeTypeMap);
    prepareMap(mCountryMap);
    prepareMap(mRoadTypeMap);
    prepareMap(mRoadMarkRuleMap);
    prepareMap(
        QUzzb);
    prepareMap(hYi15);
  }
  void ReaderXML::addAttribute(const std::string &key,
                               const std::string &value)
  {
    if (mVerbose)
      fprintf(stderr,
              "\x52\x65\x61\x64\x65\x72\x58\x4d\x4c\x3a\x3a\x61\x64\x64\x41\x74\x74\x72\x69\x62\x75\x74\x65\x3a\x20\x61\x64\x64\x69\x6e\x67\x20\x70\x61\x69\x72\x20\x3c\x25\x73\x3e\x2f\x3c\x25\x73\x3e"
              "\n",
              key.c_str(), value.c_str());
    mAttribMap.insert(std::pair<std::string, std::string>(key, value));
  }
  void ReaderXML::clearAttributes()
  {
    AttribMap::iterator VfDVC =
        mAttribMap.begin();
    while (VfDVC != mAttribMap.end())
    {
      mAttribMap.erase(VfDVC);
      VfDVC =
          mAttribMap.begin();
    }
  }
  void ReaderXML::prepareMap(StringValMap &wVomq)
  {
    StringValMap
        K2g7l;
    for (StringValMap::iterator VfDVC = wVomq.begin(); VfDVC != wVomq.end(); VfDVC++)
    {
      std::string AoOE4 = VfDVC->first;
      for (unsigned int i = 0; i < AoOE4.length(); i++)
        AoOE4
            .at(i) = tolower(AoOE4.at(i));
      K2g7l.insert(std::pair<std::string, unsigned int>(
          AoOE4, VfDVC->second));
    }
    wVomq.clear();
    wVomq = K2g7l;
  }
  const std::string &ReaderXML::
      loCase(const std::string &name)
  {
    mLcString = name;
    for (unsigned int i = 0; i < mLcString.length(); i++)
      mLcString.at(i) = tolower(mLcString.at(i));
    return mLcString;
  }
  void
  ReaderXML::A3Ua9()
  {
    if (!mRootNode || !mOffset)
      return;
    Offset *stOZ4 = reinterpret_cast<
        Offset *>(mOffset);
    mRootNode->transform(stOZ4->mX, stOZ4->mY, stOZ4->mZ, stOZ4->mH,
                         true);
  }
} // namespace OpenDrive
