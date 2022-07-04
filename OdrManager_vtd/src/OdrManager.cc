
#include <stdio.h>
#include "OdrManager.hh"
#include "OdrPosition.hh"
#include "OdrReaderXML.hh"
#include "OdrRoadData.hh"
#include "OdrParserCallback.hh"
#include "OdrRoadHeader.hh"
#include "OdrLaneSection.hh"
#include "OdrLane.hh"
#include "OdrPath.hh"
#include "OdrJuncHeader.hh"
#include "OdrProjection.hh"
#include "OdrSurfaceCRG.hh"
#include "OdrSignal.hh"
#include <assert.h>
#include <stdlib.h>
#include <iostream>
const unsigned int OpenDrive::OdrManager::LOADER_ADD_BORDERS = 1;
const unsigned int OpenDrive::OdrManager::LOADER_VERBOSE = 2;
const unsigned int OpenDrive::OdrManager::LOADER_XTEND_ADD_SIGNALS = 3;
const unsigned int OpenDrive::OdrManager::LOADER_SPLIT_JUNCTIONS = 4;
const unsigned int OpenDrive::OdrManager::LOADER_CHECK_GEOMETRY = 5;
const unsigned int OpenDrive::OdrManager::LOADER_CONTINUOUS_OBJECT_TESSELATION = 6;
const unsigned int OpenDrive::OdrManager::LOADER_RESOLVE_REPEATED_OBJECTS = 7;
const unsigned int OpenDrive::OdrManager::LOADER_IGNORE_SURFACE = 8;
const unsigned int OpenDrive::OdrManager::LOADER_INHIBIT_CURVATURE_APPROXIMATION = 9;
static bool opKzZ = false;
namespace OpenDrive
{
  RoadMark* OdrManager::getQueriedRoadMark() {
    return mPos->getQueriedRoadMark();
  }


  void OdrManager::deletePosition(Position *&oh2td)
  {
    delete oh2td;
    oh2td = 0;
  }
  OdrManager::OdrManager(bool YoRcV) : OdrManagerLite(0x20070202), mMinimizeDeltaZ(
                                                                       false),
                                       mZTolerance(2.0)
  {
    opKzZ = YoRcV;
    intro();
  }
  OdrManager::~OdrManager() {}
  void
  OdrManager::intro()
  {
    if (!opKzZ)
      return;
    fprintf(stderr,
            "\n"
            "\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x4f\x70\x65\x6e\x44\x52\x49\x56\x45\x20\x6d\x61\x6e\x61\x67\x65\x72\x20\x2d\x20\x46\x55\x4c\x4c\x20\x56\x45\x52\x53\x49\x4f\x4e\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x28\x63\x29\x32\x30\x31\x37\x20\x62\x79\x20\x56\x49\x52\x45\x53\x20\x53\x69\x6d\x75\x6c\x61\x74\x69\x6f\x6e\x73\x74\x65\x63\x68\x6e\x6f\x6c\x6f\x67\x69\x65\x20\x47\x6d\x62\x48\x2c\x20\x47\x65\x72\x6d\x61\x6e\x79\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x4e\x4f\x54\x45\x3a\x20\x54\x68\x69\x73\x20\x73\x6f\x66\x74\x77\x61\x72\x65\x20\x69\x73\x20\x70\x72\x6f\x76\x69\x64\x65\x64\x20\x41\x53\x20\x49\x53\x20\x77\x69\x74\x68\x6f\x75\x74\x20\x61\x6e\x79\x20\x77\x61\x72\x72\x61\x6e\x74\x79\x20\x6f\x72\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x70\x65\x72\x66\x6f\x72\x6d\x61\x6e\x63\x65\x20\x67\x75\x61\x72\x61\x6e\x74\x65\x65\x73\x2e\x20\x56\x49\x52\x45\x53\x20\x6d\x75\x73\x74\x20\x6e\x6f\x74\x20\x62\x65\x20\x68\x65\x6c\x64\x20\x6c\x69\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x61\x6e\x79\x20\x64\x61\x6d\x61\x67\x65\x73\x20\x69\x6e\x63\x75\x72\x72\x69\x6e\x67\x20\x66\x72\x6f\x6d\x20\x74\x68\x65\x20\x75\x73\x65\x20\x6f\x66\x20\x74\x68\x69\x73\x20\x73\x6f\x66\x74\x77\x61\x72\x65\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x41\x70\x72\x69\x6c\x20\x32\x30\x31\x36\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x46\x6f\x72\x20\x71\x75\x65\x73\x74\x69\x6f\x6e\x73\x2c\x20\x66\x65\x65\x64\x62\x61\x63\x6b\x20\x61\x6e\x64\x20\x6f\x72\x64\x65\x72\x20\x69\x6e\x66\x6f\x72\x6d\x61\x74\x69\x6f\x6e\x2c\x20\x70\x6c\x65\x61\x73\x65\x20\x72\x65\x66\x65\x72\x20\x74\x6f\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x56\x49\x52\x45\x53\x20\x53\x69\x6d\x75\x6c\x61\x74\x69\x6f\x6e\x73\x74\x65\x63\x68\x6e\x6f\x6c\x6f\x67\x69\x65\x20\x47\x6d\x62\x48\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x47\x72\x61\x73\x73\x69\x6e\x67\x65\x72\x20\x53\x74\x72\x61\x73\x73\x65\x20\x38\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x38\x33\x30\x34\x33\x20\x42\x61\x64\x20\x41\x69\x62\x6c\x69\x6e\x67\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x47\x65\x72\x6d\x61\x6e\x79\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x70\x68\x6f\x6e\x65\x20\x2b\x34\x39\x2e\x38\x30\x36\x31\x2e\x39\x33\x39\x30\x39\x33\x2d\x30\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x66\x61\x78\x20\x20\x20\x2b\x34\x39\x2e\x38\x30\x36\x31\x2e\x39\x33\x39\x30\x39\x33\x2d\x31\x33\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x6f\x70\x65\x6e\x64\x72\x69\x76\x65\x40\x76\x69\x72\x65\x73\x2e\x63\x6f\x6d\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x20\x77\x77\x77\x2e\x76\x69\x72\x65\x73\x2e\x63\x6f\x6d\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20"
            "\n");
    fprintf(stderr,
            "\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23\x23"
            "\n");
  }
  bool OdrManager::loadData(const std::string &AZJf9) { return load("", AZJf9); }
  bool OdrManager::loadFile(const std::string &name) { 
    printf("load file\n");
    return load(name); 
  }
  bool OdrManager::load(const std::string &LiJIF, const std::string &AZJf9)
  {
    printf("load file\n");
    if (mRoadData)
      delete mRoadData;
    if (LiJIF.empty() && AZJf9.empty())
      return false;
    OpenDrive::
        ReaderXML *LxeNv = new OpenDrive::ReaderXML();
    if (AZJf9.empty())
      LxeNv->setFilename(
          LiJIF);
    for (OptionMap::iterator VfDVC = mOptionMap.begin(); VfDVC != mOptionMap.end();
         VfDVC++)
    {
      if (VfDVC->first == LOADER_ADD_BORDERS)
        LxeNv->addBorderLanes(VfDVC->second);
      if (VfDVC->first == LOADER_VERBOSE)
        LxeNv->setVerbose((int)(VfDVC->second));
      if (
          VfDVC->first == LOADER_SPLIT_JUNCTIONS && (VfDVC->second == 1))
        LxeNv->addFlag(LxeNv->FLAG_SPLIT_JUNCTIONS);
      if (VfDVC->first == LOADER_CHECK_GEOMETRY && (VfDVC->second == 1))
        LxeNv->addFlag(LxeNv->FLAG_CHECK_GEOMETRY);
      if (VfDVC->first ==
          LOADER_RESOLVE_REPEATED_OBJECTS)
        LxeNv->SrdMD(VfDVC->second != 0);
      if (VfDVC->first ==
          LOADER_CONTINUOUS_OBJECT_TESSELATION)
        LxeNv->Fa4eH(VfDVC->second);
      if (VfDVC->first == LOADER_IGNORE_SURFACE)
        LxeNv->addFlag(LxeNv->NyNgk);
      if (VfDVC->first ==
              LOADER_INHIBIT_CURVATURE_APPROXIMATION &&
          (VfDVC->second == 1))
        LxeNv->addFlag(LxeNv
                           ->FLAG_INHIBIT_CURVATURE_APPROXIMATION);
    }
    if (AZJf9.empty())
      LxeNv->read();
    else
      LxeNv->read(AZJf9);
    if (!(LxeNv->getRootNode()))
      return false;
    mRoadData = RoadData::
        getInstance();
    delete LxeNv;
    return true;
  }
  bool OdrManager::addFile(const std::
                               string &name)
  {
    unsigned int flags = 0;
    OpenDrive::ReaderXML *LxeNv = new OpenDrive::
        ReaderXML();
    LxeNv->setFilename(name);
    for (OptionMap::iterator VfDVC = mOptionMap.begin(); VfDVC != mOptionMap.end(); VfDVC++)
    {
      if (VfDVC->first ==
          LOADER_XTEND_ADD_SIGNALS)
        flags |= LxeNv->FLAG_ADD_SIGNALS;
      if (VfDVC->first ==
          LOADER_ADD_BORDERS)
        LxeNv->addBorderLanes(VfDVC->second);
      if (VfDVC->first ==
          LOADER_VERBOSE)
        LxeNv->setVerbose((int)(VfDVC->second));
    }
    LxeNv->add(flags);
    delete LxeNv;
    return true;
  }
  void OdrManager::setLoaderOption(const unsigned int &option,
                                   const double &value)
  {
    std::pair<OptionMap::iterator, bool> QVavK = mOptionMap.insert(
        std::pair<unsigned int, double>(option, value));
    if (!QVavK.second)
      std::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x73\x65\x74\x4c\x6f\x61\x64\x65\x72\x4f\x70\x74\x69\x6f\x6e\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x73\x65\x74\x20\x6f\x70\x74\x69\x6f\x6e\x21"
                << std::endl;
  }
  Position *OdrManager::createPosition() { return new Position; }
  bool
  OdrManager::track2lane()
  {
    assert(mPos);
    return mPos->track2lane() == RoadQuery::
                                     RESULT_ON_ROAD;
  }
  bool OdrManager::track2curvature()
  {
    assert(mPos);
    return mPos->track2curvature() == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::lane2track()
  {
    assert(mPos);
    return mPos->lane2track() == RoadQuery::RESULT_ON_ROAD;
  }
  const double &
  OdrManager::getTrackWidth() const
  {
    assert(mPos);
    mPos->calcTrackWidth();
    return mPos
        ->getTrackWidth();
  }
  const Coord &OdrManager::getTrackAngles() const
  {
    assert(mPos);
    mPos->calcTrackAngles();
    return mPos->getTrackAngles();
  }
  const unsigned short &
  OdrManager::getRoadMark() const
  {
    assert(mPos);
    mPos->lane2roadMark();
    return mPos->getRoadMark();
  }
  void OdrManager::parseTree(ParserCallback *cb)
  {
    assert(cb);
    if (
        mRoadData->getRootNode())
    {
      OpenDrive::Node *xRNvF = mRoadData->getRootNode();
      xRNvF->parse(cb);
    }
    cb->endOfData();
  }
  Node *OdrManager::getRootNode()
  {
    if (mRoadData)
      return mRoadData->getRootNode();
    return 0;
  }
  const Node *OdrManager::getRootNode() const
  {
    if (mRoadData)
      return mRoadData->getRootNode();
    return 0;
  }
  bool OdrManager::
      inertial2track()
  {
    assert(mPos);
    mPos->setZOptimization(mMinimizeDeltaZ, mZTolerance);
    return OdrManagerLite::inertial2track();
  }
  bool OdrManager::inertial2lane()
  {
    assert(mPos);
    mPos->setZOptimization(mMinimizeDeltaZ, mZTolerance);
    return OdrManagerLite::inertial2lane();
  }
  bool OdrManager::inertial2lane(int LqMUn)
  {
    assert(mPos);
    mPos->setZOptimization(mMinimizeDeltaZ, mZTolerance);
    return mPos->inertial2lane(LqMUn) == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::inertial2lane(
      const std::string &LqMUn)
  {
    assert(mPos);
    mPos->setZOptimization(mMinimizeDeltaZ,
                           mZTolerance);
    return mPos->inertial2lane(LqMUn) == RoadQuery::RESULT_ON_ROAD;
  }
  bool
  OdrManager::inertial2laneList(bool dZZZn)
  {
    assert(mPos);
    mPos->setZOptimization(
        mMinimizeDeltaZ, mZTolerance);
    return mPos->inertial2laneList(dZZZn) == RoadQuery::
                                                 RESULT_ON_ROAD;
  }
  bool OdrManager::inertial2geo()
  {
    assert(mPos);
    return mPos->inertial2geo() == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::geo2inertial()
  {
    assert(mPos);
    return mPos->geo2inertial() == RoadQuery::RESULT_ON_ROAD;
  }
  LaneCoord::
      LaneVec &
      OdrManager::getLaneList()
  {
    assert(mPos);
    return const_cast<LaneCoord::
                          LaneVec &>(mPos->getLaneList());
  }
  RoadHeader *OdrManager::getRoadHeader()
  {
    assert(
        mPos);
    return mPos->getQueriedRoadHeader();
  }
  LaneSection *OdrManager::
      getLaneSection()
  {
    assert(mPos);
    return mPos->getQueriedLaneSection();
  }
  Lane *
  OdrManager::getLane()
  {
    assert(mPos);
    return mPos->getQueriedLane();
  }
  Elevation *
  OdrManager::getElevation()
  {
    assert(mPos);
    return mPos->getQueriedElevation();
  }
  Superelevation *OdrManager::getSuperelevation()
  {
    assert(mPos);
    return mPos->getQueriedSuperelevation();
  }
  int OdrManager::getJunctionId()
  {
    assert(mPos);
    return mPos->getJunctionId();
  }
  unsigned int OdrManager::getLaneType()
  {
    assert(mPos);
    return mPos->getLaneType();
  }
  bool OdrManager::getMaterial(std::string &o055c,
                               double &jGi0_, double &zJQyy)
  {
    assert(mPos);
    return mPos->getMaterial(o055c, jGi0_,
                             zJQyy);
  }
  bool OdrManager::getMaterial(unsigned int &code, double &jGi0_, double &zJQyy)
  {
    assert(mPos);
    return mPos->getMaterial(code, jGi0_, zJQyy);
  }
  bool OdrManager::
      intersectCircle(const double &tyz2F) { return mPos->intersectCircle(tyz2F); }
  void
  OdrManager::assignPath(Path *japsW)
  {
    assert(mPos);
    mPos->setPath(japsW);
  }
  void
  OdrManager::printPath(Path *japsW)
  {
    assert(mPos);
    japsW->print();
  }
  bool OdrManager::
      lane2path()
  {
    assert(mPos);
    return mPos->lane2path() == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::inertial2path()
  {
    assert(mPos);
    return mPos->inertial2path() ==
           RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::addPathOffset(const double &rganP,
                                 const int &xCtky, const double &gmQV0)
  {
    assert(mPos);
    return mPos->addPathOffset(
               rganP, xCtky, gmQV0) == RoadQuery::RESULT_ON_ROAD;
  }
  Path *OdrManager::createPath(const std::string &name) { return new Path(name); }
  void OdrManager::deletePath(Path *japsW)
  {
    assert(mPos);
    if (mPos->getPath() == japsW)
      mPos->setPath(0);
    delete japsW;
  }
  bool
  OdrManager::addPosToPath(const TrackCoord &HQGFE)
  {
    assert(mPos);
    if (!mPos->getPath())
    {
      std::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x50\x6f\x73\x54\x6f\x50\x61\x74\x68\x3a\x20\x6e\x6f\x20\x70\x61\x74\x68\x20\x6f\x62\x6a\x65\x63\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x6f\x62\x6a\x65\x63\x74"
                << std::endl;
      return false;
    }
    return mPos->getPath()->addWaypoint(HQGFE);
  }
  bool
  OdrManager::setPathPos(const double &BJBDA)
  {
    assert(mPos);
    if (!mPos->getPath())
    {
      std ::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x73\x65\x74\x50\x61\x74\x68\x50\x6f\x73\x3a\x20\x6e\x6f\x20\x70\x61\x74\x68\x20\x6f\x62\x6a\x65\x63\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x6f\x62\x6a\x65\x63\x74"
                 << std::endl;
      return false;
    }
    mPos->getPath()->setPos(BJBDA);
    return true;
  }
  double
  OdrManager::getPathLength()
  {
    assert(mPos);
    if (!mPos->getPath())
    {
      std::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x67\x65\x74\x50\x61\x74\x68\x4c\x65\x6e\x67\x74\x68\x3a\x20\x6e\x6f\x20\x70\x61\x74\x68\x20\x6f\x62\x6a\x65\x63\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x6f\x62\x6a\x65\x63\x74"
                << std::endl;
      return 0.0;
    }
    return mPos->getPath()->getLength();
  }
  double OdrManager::
      getPathPos()
  {
    assert(mPos);
    if (!mPos->getPath())
    {
      std::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x67\x65\x74\x50\x61\x74\x68\x50\x6f\x73\x3a\x20\x6e\x6f\x20\x70\x61\x74\x68\x20\x6f\x62\x6a\x65\x63\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x6f\x62\x6a\x65\x63\x74"
                << std::endl;
      return 0.0;
    }
    return mPos->getPath()->getS();
  }
  bool OdrManager::
      getPathFwd()
  {
    assert(mPos);
    if (!mPos->getPath())
    {
      std::cerr << "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x67\x65\x74\x50\x61\x74\x68\x46\x77\x64\x3a\x20\x6e\x6f\x20\x70\x61\x74\x68\x20\x6f\x62\x6a\x65\x63\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x20\x66\x6f\x72\x20\x63\x75\x72\x72\x65\x6e\x74\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x6f\x62\x6a\x65\x63\x74"
                << std::endl;
      return 0.0;
    }
    return mPos->getPath()->getFwd();
  }
  void OdrManager::
      copyPos(Position *nC4bv, Position *SkShA)
  {
    if (!nC4bv || !SkShA)
      return;
    *nC4bv = *SkShA;
  }
  double OdrManager::getLaneCurvature()
  {
    assert(mPos);
    mPos->lane2laneCurvature();
    return mPos->getLaneCurvature();
  }
  bool OdrManager::collectSignals(bool JN4Nc,
                                  const double &F5cuE)
  {
    assert(mPos);
    return mPos->collectSignals(JN4Nc, F5cuE);
  }

  bool OdrManager::collectObjects(bool JN4Nc,
                                  const double &F5cuE)
  {
      assert(mPos);
      return mPos->collectObjects(JN4Nc, F5cuE);
  }

  int OdrManager::getCollectionSize()
  {
    assert(mPos);
    return mPos->getCollectionSize();
  }
  bool OdrManager::getCollectionInfo(const int &id, double &dist, Node *&node)
  {
    assert(
        mPos);
    return mPos->getCollectionInfo(id, dist, node);
  }
  bool OdrManager::track2validTrack()
  {
    assert(mPos);
    return mPos->track2validTrack() == RoadQuery::
                                           RESULT_ON_ROAD;
  }
  bool OdrManager::lane2validLane()
  {
    assert(mPos);
    return mPos->lane2validLane() == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::calcTrackAnglesDot()
  {
    assert(mPos);
    mPos->calcTrackAnglesDot();
    return true;
  }
  const double &OdrManager ::getDhDs() const
  {
    assert(mPos);
    return mPos->getDhDs();
  }
  const double &OdrManager::
      getDpDs() const
  {
    assert(mPos);
    return mPos->getDpDs();
  }
  const double &OdrManager::
      getDrDs() const
  {
    assert(mPos);
    return mPos->getDrDs();
  }
  const double &OdrManager::
      getDzDs() const
  {
    assert(mPos);
    return mPos->getDzDs();
  }
  const double &OdrManager::
      getD2zDs() const
  {
    assert(mPos);
    return mPos->getD2zDs();
  }
  const double &OdrManager::
      getDzDt() const
  {
    assert(mPos);
    return mPos->getDzDt();
  }
  bool OdrManager::
      calcCrossSection()
  {
    assert(mPos);
    return mPos->calcCrossSection();
  }
  int OdrManager ::getCrossSectionSize()
  {
    assert(mPos);
    return mPos->getCrossSectionSize();
  }
  bool
  OdrManager::getCrossSectionLaneInfo(const int &id, int &laneId, int &zgDYE, double &qfK7D)
  {
    assert(mPos);
    return mPos->getCrossSectionLaneInfo(id, laneId, zgDYE, qfK7D);
  }
  bool OdrManager::track2inertialAngDotCrossSec()
  {
    assert(mPos);
    return mPos->track2inertialAngDotCrossSec() == RoadQuery::RESULT_ON_ROAD;
  }
  bool OdrManager::
      getNextJunction(bool JN4Nc, JuncHeader *&jHdr, double &ioAAo)
  {
    assert(mPos);
    return mPos->getNextJunction(JN4Nc, jHdr, ioAAo);
  }
  int OdrManager::getJunctionInfoSize()
  {
    assert(mPos);
    return mPos->getJunctionInfoSize();
  }
  int OdrManager::getRoadType()
  {
    assert(mPos);
    return mPos->getRoadType();
  }
  double OdrManager::getLaneSpeed()
  {
    assert(mPos);
    double kMjGG;
    if (mPos->getLaneSpeed(kMjGG))
      return kMjGG;
    return -1.0;
  }
  bool OdrManager::getJunctionInfo(const unsigned int &hFNbl, RoadHeader *&inRoad,
                                   RoadHeader *&connRoad, double &turnAngle, int &XWWHl, int &ckJEX)
  {
    assert(mPos);
    JuncHeader *jHdr;
    return mPos->getJunctionInfo(hFNbl, jHdr, inRoad, connRoad,
                                 turnAngle, XWWHl, ckJEX);
  }
  const double &OdrManager::getLaneCurvatureDot() const
  {
    assert(mPos);
    return mPos->getLaneCurvatureDot();
  }
  const double &OdrManager::getLaneCurvatureVert() const
  {
    assert(mPos);
    return mPos->getLaneCurvatureVert();
  }
  const double &OdrManager::getLaneCurvatureVertDot() const
  {
    assert(mPos);
    return mPos->getLaneCurvatureVertDot();
  }
  void OdrManager::print(Position *oxWEh)
  {
    if (!oxWEh)
      return;
    oxWEh->print();
  }
  void OdrManager::print(int NS8oU) 
  { 
    OdrManagerLite::print(NS8oU); 
  }
  void OdrManager::setSurfaceScale(const double &wzKzI)
  {
    mSurfaceScale = wzKzI;
    if (!mPos)
      return;
    mPos->setSurfaceScale(wzKzI);
  }
  bool OdrManager::inTunnel()
  {
    if (!mPos)
      return false;
    return mPos->inTunnel();
  }
  bool OdrManager::onBridge()
  {
    if (!mPos)
      return false;
    return mPos->onBridge();
  }
  const GeoCoord &OdrManager::getGeoPos()
      const
  {
    assert(mPos);
    return mPos->getGeoPos();
  }
  void OdrManager::setPos(const GeoCoord &value)
  {
    assert(mPos);
    mPos->setPos(value);
  }
  void OdrManager::setGeoPos(
      const double &auc9o, const double &smK7U, const double &hkK5C)
  {
    assert(mPos);
    mPos->setGeoPos(auc9o, smK7U, hkK5C);
  }
  void OdrManager::setContactPatchDimension(const double &length, const double &width)
  {
    assert(mPos);
    mPos->setContactPatchDimension(
        length, width);
  }
  void OdrManager::useLaneHeight(bool F744N)
  {
    assert(mPos);
    mPos->useLaneHeight(F744N);
  }
  bool OdrManager::projectionSetProj4Inertial(const std::string &gBhbI)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x50\x72\x6f\x6a\x34\x49\x6e\x65\x72\x74\x69\x61\x6c\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    if (!mRoadData->getProjection())
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x50\x72\x6f\x6a\x34\x49\x6e\x65\x72\x74\x69\x61\x6c\x3a\x20\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x20\x63\x6c\x61\x73\x73\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    mRoadData->getProjection()->setProj4Inertial(gBhbI);
    return true;
  }
  bool OdrManager::projectionSetProj4Geo(const std::string &gBhbI)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x50\x72\x6f\x6a\x34\x47\x65\x6f\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    if (!mRoadData->getProjection())
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x50\x72\x6f\x6a\x34\x47\x65\x6f\x3a\x20\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x20\x63\x6c\x61\x73\x73\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    mRoadData->getProjection()->setProj4Geo(gBhbI);
    return true;
  }
  bool
  OdrManager::projectionSetOGCWKT(const std::string &gBhbI)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x4f\x47\x43\x57\x4b\x54\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    if (!mRoadData->getProjection())
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x53\x65\x74\x4f\x47\x43\x57\x4b\x54\x3a\x20\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x20\x63\x6c\x61\x73\x73\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    mRoadData->getProjection()->setOGCWKT(gBhbI);
    return true;
  }
  void
  OdrManager::setOriginInertial(const Coord &offset)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return;
    }
    if (!mRoadData->getProjection())
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x20\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x20\x63\x6c\x61\x73\x73\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return;
    }
    mRoadData->getProjection()->setOriginInertial(offset);
  }
  const Coord &
  OdrManager::getOriginInertial() const
  {
    static const Coord U627e;
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return U627e;
    }
    if (!mRoadData->getProjection())
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x20\x70\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x20\x63\x6c\x61\x73\x73\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return U627e;
    }
    return mRoadData->getProjection()->getOriginInertial();
  }
  bool
  OdrManager::addSurfaceCRG(const unsigned int &I8pOy, const std::string &kQyVK, const double &xKENU, const double &sEnd, const unsigned char &c9NG2, const unsigned int &nU5a4, const double &SV89_, const double &rg0Co, const double &dBa0u, const double &OWCFd, const double &vUAeR, const unsigned short &TCz3u)
  {
    if (!mRoadData)
    {
      fprintf(
          stderr,
          "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
          "\n");
      return false;
    }
    RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mRoadData->getNodeFromId(ODR_OPCODE_ROAD_HEADER, I8pOy));
    if (!M8Gux)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x6f\x61\x64\x20\x3c\x25\x64\x3e\x20\x6e\x6f\x74\x20\x66\x6f\x75\x6e\x64\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n",
              I8pOy);
      return false;
    }
    return addSurfaceCRG(M8Gux, kQyVK, xKENU, sEnd, c9NG2, nU5a4,
                         SV89_, rg0Co, dBa0u, OWCFd, vUAeR, TCz3u);
  }
  bool OdrManager::addSurfaceCRG(const std::
                                     string &I8pOy,
                                 const std::string &kQyVK, const double &xKENU, const double &sEnd, const unsigned char &c9NG2, const unsigned int &nU5a4, const double &SV89_, const double &rg0Co, const double &dBa0u, const double &OWCFd, const double &vUAeR, const unsigned short &TCz3u)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mRoadData->getNodeFromId(ODR_OPCODE_ROAD_HEADER, I8pOy));
    if (!M8Gux)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x75\x72\x66\x61\x63\x65\x43\x52\x47\x3a\x20\x72\x6f\x61\x64\x20\x3c\x25\x73\x3e\x20\x6e\x6f\x74\x20\x66\x6f\x75\x6e\x64\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n",
              I8pOy.c_str());
      return false;
    }
    return addSurfaceCRG(M8Gux, kQyVK, xKENU, sEnd, c9NG2,
                         nU5a4, SV89_, rg0Co, dBa0u, OWCFd, vUAeR, TCz3u);
  }
  bool OdrManager::addSurfaceCRG(
      RoadHeader *M8Gux, const std::string &kQyVK, const double &xKENU, const double &sEnd,
      const unsigned char &c9NG2, const unsigned int &nU5a4, const double &SV89_, const double &rg0Co, const double &dBa0u, const double &OWCFd, const double &vUAeR, const unsigned short &TCz3u)
  {
    if (!M8Gux)
      return false;
    OpenDrive::SurfaceCRG *WiKMs = new OpenDrive::SurfaceCRG();
    WiKMs->mFileName = kQyVK;
    WiKMs->mDir = c9NG2;
    WiKMs->mS = xKENU;
    WiKMs->mSEnd = sEnd;
    WiKMs->mMode = nU5a4;
    WiKMs->mSOffset = SV89_;
    WiKMs->mTOffset =
        rg0Co;
    WiKMs->mZOffset = dBa0u;
    WiKMs->mZScale = OWCFd;
    WiKMs->mHOffset = vUAeR;
    WiKMs->mPurpose = TCz3u;
    WiKMs->enableGlobalSearch();
    if (WiKMs->mDir == OpenDrive::
                           ODR_DIRECTION_MINUS)
      WiKMs->setInverseDirection();
    M8Gux->addChild(WiKMs);
    WiKMs->calcPrepareData();
    M8Gux->calcPrepareData();
    return true;
  }
  bool OdrManager::
      addSignal(const double &fmKl4, const double &KCvlU, const double &ELnJA, const float &wtTkN, const float &yfy4C, const float &ykDhG, const std::string &id, const std::string &name, bool XTxq5, const std::string &c9NG2, const std::string &LrIej, const std::string &type, const std::string &ZO9x4, const double &value, unsigned int state, const std::string &puAxp, const float &FwbGX, const float &width, const std::string &_2ur1)
  {
    if (!mRoadData)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x69\x67\x6e\x61\x6c\x3a\x20\x72\x6f\x61\x64\x20\x64\x61\x74\x61\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x61\x76\x61\x69\x6c\x61\x62\x6c\x65\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n");
      return false;
    }
    Position KauWq;
    KauWq.setPos(Coord(fmKl4, KCvlU, ELnJA, wtTkN, yfy4C,
                       ykDhG));
    if ((KauWq.inertial2track()) != RoadQuery::RESULT_ON_ROAD)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x69\x67\x6e\x61\x6c\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x66\x69\x6e\x64\x20\x72\x6f\x61\x64\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x66\x6f\x72\x20\x67\x69\x76\x65\x6e\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e"
              "\n");
      return false;
    }
    RoadHeader *M8Gux = reinterpret_cast<RoadHeader *>(mRoadData->getNodeFromId(ODR_OPCODE_ROAD_HEADER, KauWq.getTrackPos().getTrackIdAsString()));
    if (!M8Gux)
    {
      fprintf(stderr,
              "\x4f\x64\x72\x4d\x61\x6e\x61\x67\x65\x72\x3a\x3a\x61\x64\x64\x53\x69\x67\x6e\x61\x6c\x3a\x20\x72\x6f\x61\x64\x20\x3c\x25\x73\x3e\x20\x6e\x6f\x74\x20\x66\x6f\x75\x6e\x64\x2c\x20\x63\x61\x6e\x6e\x6f\x74\x20\x70\x72\x6f\x63\x65\x65\x64\x2e"
              "\n",
              KauWq.getTrackPos().getTrackIdAsString().c_str());
      return false;
    }
    ReaderXML F3vnM;
    Signal *OLyL5 = new OpenDrive::Signal();
    OLyL5->mS = KauWq.getTrackPos().getS();
    OLyL5
        ->mT = KauWq.getTrackPos().getT();
    OLyL5->mId = atoi(id.c_str());
    OLyL5->mIdAsString =
        id;
    OLyL5->mName = name;
    OLyL5->mDynamic = XTxq5;
    OLyL5->mDir = (c9NG2 == "\x2b") ? ODR_DIRECTION_PLUS : ((c9NG2 == "\x2d" ? ODR_DIRECTION_MINUS : ODR_DIRECTION_NONE));
    OLyL5->mZ = KauWq.getTrackPos().getZ();
    OLyL5->mCountry = F3vnM.getOpcodeFromCountry(
        LrIej);
    OLyL5->mType = atoi(type.c_str());
    OLyL5->mSubType = atoi(ZO9x4.c_str());
    OLyL5
        ->mValue = value;
    OLyL5->mUnit = F3vnM.getOpcodeFromUnit(puAxp);
    OLyL5->mHeight = FwbGX;
    OLyL5->mWidth = width;
    OLyL5->mText = _2ur1;
    OLyL5->mOffsetHdg = KauWq.getTrackPos().getH();
    OLyL5->mPitch = KauWq.getTrackPos().getP();
    OLyL5->mRoll = KauWq.getTrackPos()
                       .getR();
    OLyL5->mState = state;
    OLyL5->mReadability = 1.0;
    OLyL5->mOcclusion = 0.0;
    OLyL5
        ->mMinLane = -99;
    OLyL5->mMaxLane = 99;
    M8Gux->addChild(OLyL5);
    OLyL5->calcPrepareData();
    M8Gux->calcPrepareData();
    return true;
  }
  bool OdrManager::attach()
  {
    if (!mRoadData)
    {
      mRoadData = RoadData::getInstance();
      mAttachedOnly = (mRoadData != 0);
    }
    return mAttachedOnly;
  }
  void OdrManager::setZOptimization(bool F744N, const float &AiTnB)
  {
    mMinimizeDeltaZ = F744N;
    mZTolerance = AiTnB;
  }
} // namespace OpenDrive
