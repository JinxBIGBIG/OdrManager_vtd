
#include <math.h>
#include "OdrRoadQuery.hh"
#include "OdrHeader.hh"
#include "OdrCrossfall.hh"
#include "OdrElevation.hh"
#include "OdrLateralShape.hh"
#include "OdrGeoHeader.hh"
#include "OdrGeoNode.hh"
#include "OdrSuperelevation.hh"
#include "OdrLaneSection.hh"
#include "OdrLaneMaterial.hh"
#include "OdrLaneSpeed.hh"
#include "OdrLaneWidth.hh"
#include "OdrLaneOffset.hh"
#include "OdrLaneLink.hh"
#include "OdrLaneHeight.hh"
#include "OdrRoadLink.hh"
#include "OdrRoadHeader.hh"
#include "OdrRoadMark.hh"
#include "OdrSignal.hh"
#include "OdrSignalRef.hh"
#include "OdrJuncHeader.hh"
#include "OdrJuncLink.hh"
#include "OdrJuncLaneLink.hh"
#include "OdrRoadType.hh"
#include "OdrRoadSpeed.hh"
#include "OdrObject.hh"
#include "OdrSurfaceCRG.hh"
#include "OdrRailroadSwitch.hh"
#include "OdrTunnel.hh"
#include "OdrBridge.hh"
#include "OdrBbox.hh"
#include "OdrProjection.hh"
#include "OdrLaneBorder.hh"
#include <iostream>
#include <algorithm>
#include <stdio.h>
namespace OpenDrive
{
  const double RoadQuery::maxTrackDist = 20.0;
  const double
      RoadQuery::sMaxDeltaS = 1.e-5;
  RoadQuery::RoadQuery(const std::string &name) : mName(
                                                      name),
                                                  mRoadData(NULL), mCurvature(0.0), mLaneCurvature(0.0), mLaneCurvatureDot(0.0), mLaneCurvatureVert(0.0), mLaneCurvatureVertDot(0.0), mLaneDeltaHdg(0.0),
                                                  mSTolerance(1.0e-3), mTest(false), mExplicitRoadData(false), mDebugMask(0x0),
                                                  mMaxTrackDist(maxTrackDist), mSurfaceScale(1.0), mAllowLaneTypeRail(false),
                                                  mAllowLaneTypeCar(true), mContactPatchLength(0.0), mContactPatchWidth(0.0),
                                                  mUseContactPatch(false), mUseLaneHeight(true), mMinimizeDeltaZ(false), mZTolerance(
                                                                                                                             2.0f),
                                                  mOptimumSolution(true) { checkForRoadData(); }
  RoadQuery::~RoadQuery()
  {
    mNodeAccessMap.clear();
  }
  int RoadQuery::track2inertial(Coord &Tlj7t, const TrackCoord &HQGFE)
  {
    mMaxTrackPos = 0.0;
    RoadHeader *REKut = trackId2Node(HQGFE);
    if (!REKut)
      return RESULT_ERROR;
    GeoHeader *xjOaT = track2geoHeader(REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    mMaxTrackPos = REKut->mLength;
    GeoNode *pWj2K =
        reinterpret_cast<GeoNode *>(xjOaT->getChild());
    double x;
    double y;
    double aOzQT;
    double rkiXc = HQGFE.getT();
    double hgSHv;
    if (getRoll(REKut, HQGFE, hgSHv) ==
        RESULT_ON_ROAD)
    {
      rkiXc *= cos(hgSHv);
      Tlj7t.setR(hgSHv);
    }
    double ux1Mc = s2tolS(HQGFE.getS(), REKut);
    if (ux1Mc == REKut->mLength)
      ux1Mc -= mSTolerance;
    if (!pWj2K->st2xyh(
            ux1Mc, rkiXc, x, y, aOzQT))
      return RESULT_ERROR;
    Tlj7t.setX(x);
    Tlj7t.setY(y);
    Tlj7t.setH(aOzQT + HQGFE.getH());
    mCurvature = geoNode2curvature(pWj2K, REKut, ux1Mc);
    return track2InertialPitchAndZ(REKut, Tlj7t, HQGFE);
  }
  int RoadQuery::track2inertialSimple(
      Coord &Tlj7t, const TrackCoord &HQGFE, RoadHeader *REKut)
  {
    if (!REKut)
      return RESULT_ERROR;
    GeoHeader *xjOaT = track2geoHeader(REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
    double x;
    double y;
    double aOzQT;
    double rkiXc = HQGFE.getT();
    double ux1Mc = s2tolS(
        HQGFE.getS(), REKut);
    if (ux1Mc == REKut->mLength)
      ux1Mc -= mSTolerance;
    if (!pWj2K->st2xyh(ux1Mc, rkiXc, x, y, aOzQT))
      return RESULT_ERROR;
    Tlj7t.setX(x);
    Tlj7t.setY(y);
    return RESULT_ON_ROAD;
  }
  int RoadQuery::inertial2track(const Coord &wiBPl,
                                TrackCoord &HQGFE, const double &Yrh_j)
  {
    RoadHeader *hk8w1 = NULL;
    RoadHeader *XmGAm = NULL;
    RoadHeader *P2zEG = NULL;
    GeoHeader *c11cN = NULL;
    GeoHeader *X3EwH = NULL;
    bool Qn7tB =
        false;
    bool eVFxU = false;
    unsigned int wGf2c = 0;
    if (!checkForRoadData())
      return RESULT_ERROR;
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(mHierPos.findNode(
        ODR_OPCODE_ROAD_HEADER));
    bool q73fi = REKut != 0;
    Qn7tB = q73fi;
    if (!q73fi)
      REKut =
          reinterpret_cast<RoadHeader *>(mRoadData->findFirstNode(ODR_OPCODE_ROAD_HEADER));
    while (REKut)
    {
      GeoHeader *xjOaT = 0;
      if (q73fi)
        xjOaT = reinterpret_cast<GeoHeader *>(
            mHierPos.findNode(ODR_OPCODE_GEO_HEADER));
      bool e_m_o = xjOaT != 0;
      eVFxU = e_m_o;
      if (!e_m_o)
      {
        if (REKut->inBoundingBox(wiBPl.getX(), wiBPl.getY()))
          xjOaT = REKut->getFirstGeoHeader();
        else
          xjOaT = 0;
      }
      while (xjOaT)
      {
        GeoNode *pWj2K = reinterpret_cast<
            GeoNode *>(xjOaT->getChild());
        if (!pWj2K)
          return RESULT_ERROR;
        if ((xjOaT->inBoundingBox(wiBPl.getX(), wiBPl.getY()) && pWj2K->containsPos(wiBPl.getX(), wiBPl.getY())) && (xjOaT->mSEnd > Yrh_j))
        {
          double BJBDA;
          double rkiXc;
          bool _CPxI = pWj2K->xy2st(wiBPl.getX(), wiBPl.getY(), BJBDA, rkiXc);
          if (_CPxI && fabs(rkiXc) <= mMaxTrackDist && (BJBDA >= Yrh_j))
          {
            HQGFE.setTrackId(REKut->mId);
            HQGFE.setTrackId(
                REKut->mIdAsString);
            HQGFE.setS(BJBDA);
            mHierPos.keepNode(REKut);
            mHierPos.keepNode(xjOaT);
            double aOzQT;
            pWj2K->s2h(BJBDA, aOzQT);
            double hgSHv = 0.0;
            if (getRoll(REKut,
                        HQGFE, hgSHv) == RESULT_ON_ROAD)
            {
              rkiXc /= cos(hgSHv);
            }
            HQGFE.setT(rkiXc);
            mCurvature =
                geoNode2curvature(pWj2K, REKut, HQGFE.getS());
            double s4E1B;
            double hkK5C;
            int result;
            if ((result = trackAndRoll2PitchAndZ(REKut, HQGFE, hgSHv, s4E1B, hkK5C)) !=
                RESULT_ON_ROAD)
              return result;
            mFootPoint.set(wiBPl.getX(), wiBPl.getY(), hkK5C,
                           aOzQT, s4E1B, hgSHv);
            HQGFE.setZ(wiBPl.getZ() - hkK5C);
            HQGFE.setP(wiBPl.getP() - s4E1B);
            HQGFE.setR(wiBPl.getR() - hgSHv);
            HQGFE.setH(wiBPl.getH() - aOzQT);
            return result;
          }
        }
        if (e_m_o)
        {
          if (eVFxU)
          {
            c11cN = reinterpret_cast<GeoHeader *>(xjOaT->getLeft());
            X3EwH =
                reinterpret_cast<GeoHeader *>(xjOaT->getRight());
            eVFxU = false;
          }
          if (X3EwH)
          {
            xjOaT =
                X3EwH;
            X3EwH = 0;
          }
          else if (c11cN)
          {
            xjOaT = c11cN;
            c11cN = 0;
          }
          else
          {
            e_m_o = false;
            xjOaT = REKut
                        ->getFirstGeoHeader();
          }
        }
        else
          xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getRight());
      }
      if (q73fi)
      {
        e_m_o = false;
        if (Qn7tB)
        {
          hk8w1 = REKut->getPredecessor();
          XmGAm = REKut->getSuccessor();
          P2zEG = REKut;
          Qn7tB = false;
        }
        if (XmGAm)
        {
          REKut = XmGAm;
          XmGAm = 0;
        }
        else if (
            hk8w1)
        {
          REKut = hk8w1;
          hk8w1 = 0;
        }
        else if (P2zEG->getNeighbor(wGf2c))
        {
          REKut = P2zEG->getNeighbor(wGf2c++);
        }
        else
        {
          q73fi = false;
          REKut = reinterpret_cast<RoadHeader *>(
              mRoadData->findFirstNode(ODR_OPCODE_ROAD_HEADER));
        }
      }
      else
        REKut = reinterpret_cast<
            RoadHeader *>(REKut->getRight());
    }
    return RESULT_NOT_ON_ROAD;
  }
  int RoadQuery::
      inertial2lane(const Coord &wiBPl, LaneCoord &Hv9fs)
  {
    std::vector<RoadHeader *> LOM4N;
    LOM4N.clear();
    return inertial2lane(wiBPl, Hv9fs, LOM4N);
  }
  int RoadQuery::
      inertial2lane(const Coord &wiBPl, LaneCoord::LaneVec &D7gpt, bool dZZZn)
  {
    bool NkGzn =
        false;
    if (!dZZZn)
      NkGzn = mHierPos.findNode(ODR_OPCODE_ROAD_HEADER) != 0;
    D7gpt.clear();
    if (!checkForRoadData())
      return RESULT_ERROR;
    LaneCoord Hv9fs;
    int result =
        inertial2lane(wiBPl, Hv9fs);
    if (result != RESULT_ON_ROAD)
    {
      return result;
    }
    D7gpt.push_back(Hv9fs);
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(mHierPos.findNode(ODR_OPCODE_ROAD_HEADER));
    if (!REKut)
      return RESULT_ERROR;
    if (!NkGzn)
    {
      RoadHeader *c5SkH = reinterpret_cast<RoadHeader *>(mRoadData->findFirstNode(
          ODR_OPCODE_ROAD_HEADER));
      while (c5SkH)
      {
        if (c5SkH != REKut)
        {
          if (c5SkH->inBoundingBox(
                  wiBPl.getX(), wiBPl.getY()))
          {
            GeoHeader *xjOaT = c5SkH->getFirstGeoHeader();
            while (
                xjOaT)
            {
              LaneCoord Hv9fs;
              if (inertial2lane(c5SkH, xjOaT, wiBPl, Hv9fs, false) ==
                  RESULT_ON_ROAD)
              {
                D7gpt.push_back(Hv9fs);
                break;
              }
              xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getRight());
            }
          }
        }
        c5SkH = reinterpret_cast<RoadHeader *>(c5SkH->getRight());
      }
    }
    else
    {
      if (!(REKut->getJunction()))
      {
        return result;
      }
      mHierPos.backup();
      JuncHeader *
          rQQS4 = reinterpret_cast<JuncHeader *>(REKut->getJunction());
      JuncLink *hYfdO =
          reinterpret_cast<JuncLink *>(rQQS4->getChild(ODR_OPCODE_JUNCTION_LINK));
      while (
          hYfdO)
      {
        RoadHeader *c5SkH = hYfdO->mConnectingRoad;
        if (c5SkH && c5SkH != REKut)
        {
          if (c5SkH
                  ->inBoundingBox(wiBPl.getX(), wiBPl.getY()))
          {
            GeoHeader *xjOaT = c5SkH->getFirstGeoHeader();
            while (xjOaT)
            {
              LaneCoord Hv9fs;
              if (inertial2lane(c5SkH, xjOaT,
                                wiBPl, Hv9fs, false) == RESULT_ON_ROAD)
              {
                D7gpt.push_back(Hv9fs);
                break;
              }
              xjOaT =
                  reinterpret_cast<GeoHeader *>(xjOaT->getRight());
            }
          }
        }
        hYfdO = reinterpret_cast<
            JuncLink *>(hYfdO->getRight());
      }
      mHierPos.restore();
    }
    if (D7gpt.size())
      return RESULT_ON_ROAD;
    return RESULT_NOT_ON_ROAD;
  }
  int RoadQuery::inertial2lane(const Coord &wiBPl, LaneCoord &Hv9fs, std::vector<RoadHeader *> &LOM4N, bool Wy_Hs)
  {
    RoadHeader *hk8w1 = NULL;
    RoadHeader *XmGAm = NULL;
    RoadHeader *P2zEG = NULL;
    GeoHeader *
        c11cN = NULL;
    GeoHeader *X3EwH = NULL;
    bool Qn7tB = false;
    bool eVFxU = false;
    unsigned int
        wGf2c = 0;
    bool _HwNc = false;
    bool e_m_o = false;
    static bool Y3vvh = false;
    std::vector<
        RoadHeader *>::iterator gRJoU = LOM4N.begin();
    mOptimumSolution = true;
    if (!checkForRoadData())
      return RESULT_ERROR;
    RoadHeader *REKut = reinterpret_cast<
        RoadHeader *>(mHierPos.findNode(ODR_OPCODE_ROAD_HEADER));
    bool q73fi = Wy_Hs && (REKut != 0);
    if (Y3vvh)
    {
      fprintf(stderr,
              "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x75\x73\x69\x6e\x67\x20\x72\x65\x67\x69\x73\x74\x65\x72\x65\x64\x20\x72\x6f\x61\x64\x20\x3d\x20\x25\x64\x2c\x20\x72\x6f\x61\x64\x56\x65\x63\x2e\x73\x69\x7a\x65\x28\x29\x20\x3d\x20\x25\x64"
              "\n",
              q73fi, (int)LOM4N.size());
      if (q73fi)
        fprintf(stderr,
                "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x72\x6f\x61\x64\x20\x3d\x20\x25\x64"
                "\n",
                REKut->mId);
    }
    if (q73fi && LOM4N.size())
    {
      while (gRJoU != LOM4N.end())
      {
        if ((*gRJoU) ==
            REKut)
          break;
        gRJoU++;
      }
      if (gRJoU == LOM4N.end())
      {
        REKut = 0;
        q73fi = false;
      }
      gRJoU = LOM4N.begin();
    }
    Qn7tB = q73fi;
    if (q73fi)
      e_m_o = true;
    else
    {
      if (LOM4N.size())
        REKut = *gRJoU;
      else
        REKut = reinterpret_cast<RoadHeader *>(mRoadData->findFirstNode(
            ODR_OPCODE_ROAD_HEADER));
    }
    while (REKut)
    {
      if (Y3vvh)
        fprintf(stderr,
                "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x72\x6f\x61\x64\x20\x25\x64"
                "\n",
                REKut->mId);
      GeoHeader *xjOaT = 0;
      if (q73fi && e_m_o)
        xjOaT = reinterpret_cast<GeoHeader *>(mHierPos.findNode(ODR_OPCODE_GEO_HEADER));
      e_m_o = xjOaT != 0;
      eVFxU = e_m_o;
      if (!e_m_o)
      {
        if (REKut->inBoundingBox(wiBPl.getX(), wiBPl.getY()))
          xjOaT = REKut->getFirstGeoHeader();
        else
          xjOaT = 0;
        if (Y3vvh)
        {
          fprintf(stderr,
                  "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x20\x25\x73\x20\x69\x6e\x20\x62\x6f\x75\x6e\x64\x69\x6e\x67\x20\x62\x6f\x78\x20\x6f\x66\x20\x72\x6f\x61\x64\x20\x25\x64"
                  "\n",
                  xjOaT ? "\x69\x73" : "\x69\x73\x20\x6e\x6f\x74", REKut->mId);
          if (REKut->getBoundingBox())
          {
            fprintf(stderr,
                    "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x20\x70\x6f\x73\x3a"
                    "\n");
            wiBPl.print();
            double pxzMQ;
            double WDWra;
            double kvKtF;
            double dQKLM;
            REKut->getBoundingBox()->get(pxzMQ, WDWra, kvKtF, dQKLM);
            fprintf(stderr,
                    "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x62\x6f\x75\x6e\x64\x69\x6e\x67\x20\x62\x6f\x78\x20\x25\x2e\x33\x66\x20\x2f\x20\x25\x2e\x33\x66\x20\x2d\x2d\x20\x25\x2e\x33\x66\x20\x2f\x20\x25\x2e\x33\x66"
                    "\n",
                    pxzMQ, WDWra, kvKtF, dQKLM);
          }
        }
      }
      while (xjOaT)
      {
        if (inertial2lane(REKut, xjOaT, wiBPl,
                          Hv9fs) == RESULT_ON_ROAD)
        {
          _HwNc = true;
          Lane *lane = reinterpret_cast<Lane *>(mHierPos.findNode(ODR_OPCODE_LANE));
          if (Y3vvh)
          {
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x6c\x61\x6e\x65\x20\x70\x6f\x73\x69\x74\x69\x6f\x6e\x3a"
                    "\n");
            Hv9fs.print();
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x6d\x41\x6c\x6c\x6f\x77\x4c\x61\x6e\x65\x54\x79\x70\x65\x43\x61\x72\x20\x3d\x20\x25\x64\x2c\x20\x6d\x41\x6c\x6c\x6f\x77\x4c\x61\x6e\x65\x54\x79\x70\x65\x52\x61\x69\x6c\x20\x3d\x20\x25\x64"
                    "\n",
                    mAllowLaneTypeCar, mAllowLaneTypeRail);
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x66\x6f\x75\x6e\x64\x20\x6c\x61\x6e\x65\x2c\x20\x69\x73\x20\x64\x72\x69\x76\x65\x61\x62\x6c\x65\x20\x3d\x20\x25\x64\x2c\x20\x72\x6f\x61\x64\x56\x65\x63\x2e\x73\x69\x7a\x65\x28\x29\x20\x3d\x20\x25\x7a\x75\x2c\x20\x75\x73\x69\x6e\x67\x52\x65\x67\x69\x73\x74\x65\x72\x65\x64\x52\x6f\x61\x64\x20\x3d\x20\x25\x64\x2c\x20\x74\x79\x70\x65\x20\x3d\x20\x25\x64\x2c\x20\x69\x64\x20\x3d\x20\x25\x64"
                    "\n",
                    lane->isDriveable(mAllowLaneTypeCar, mAllowLaneTypeRail), LOM4N.size(), q73fi, lane->mType, lane->mId);
          }
          bool Fg3xB = true;
#define X7icG
#ifdef X7icG
          if (mMinimizeDeltaZ)
          {
            double hkK5C = 0.0;
            if (!Ekmkk(REKut, Hv9fs, hkK5C))
            {
              Elevation *
                  j6B6g = s2elevation(REKut, s2tolS(Hv9fs.getS(), REKut));
              if (j6B6g)
                hkK5C = j6B6g->s2z(
                    s2tolS(Hv9fs.getS(), REKut));
              LateralShape *qDMed = EBWwH(REKut, s2tolS(Hv9fs.getS(), REKut), Hv9fs.getT());
              if (qDMed)
              {
                double hgSHv = 0.0;
                getRoll(REKut, Hv9fs, hgSHv);
                hkK5C += cos(hgSHv) * (qDMed->st2z(s2tolS(Hv9fs.getS(), REKut), Hv9fs.getT()));
              }
            }
            Fg3xB = fabs(hkK5C - wiBPl.getZ()) <= mZTolerance;
          }
#endif
          if (Fg3xB)
          {
            if (q73fi && lane->isDriveable(mAllowLaneTypeCar, mAllowLaneTypeRail))
              return RESULT_ON_ROAD;
            if (!q73fi && lane->isDriveable(mAllowLaneTypeCar,
                                            mAllowLaneTypeRail))
              return RESULT_ON_ROAD;
          }
          mHierPos.backup();
        }
        if (e_m_o)
        {
          if (eVFxU)
          {
            c11cN = reinterpret_cast<GeoHeader *>(xjOaT->getLeft());
            X3EwH = reinterpret_cast<
                GeoHeader *>(xjOaT->getRight());
            eVFxU = false;
          }
          if (X3EwH)
          {
            xjOaT = X3EwH;
            X3EwH = 0;
          }
          else if (c11cN)
          {
            xjOaT = c11cN;
            c11cN = 0;
          }
          else
          {
            e_m_o = false;
            xjOaT = REKut->getFirstGeoHeader();
          }
        }
        else
          xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getRight());
      }
      if (q73fi && !LOM4N.size())
      {
        e_m_o = false;
        if (Qn7tB)
        {
          hk8w1 = REKut->getPredecessor();
          XmGAm = REKut->getSuccessor();
          P2zEG = REKut;
          Qn7tB = false;
        }
        if (XmGAm)
        {
          REKut = XmGAm;
          XmGAm = 0;
          if (0)
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x73\x75\x63\x63\x65\x73\x73\x6f\x72\x2c\x20\x70\x74\x72\x20\x3d\x20\x25\x70\x2c\x20\x69\x64\x20\x3d\x20\x25\x64\x20"
                    "\n",
                    REKut, REKut ? REKut->mId : -1);
        }
        else if (hk8w1)
        {
          REKut = hk8w1;
          hk8w1 = 0;
          if (0)
            fprintf(
                stderr,
                "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x70\x72\x65\x64\x65\x63\x65\x73\x73\x6f\x72\x2c\x20\x70\x74\x72\x20\x3d\x20\x25\x70\x2c\x20\x69\x64\x20\x3d\x20\x25\x64\x20"
                "\n",
                REKut, REKut ? REKut->mId : -1);
        }
        else if (P2zEG->getNeighbor(wGf2c))
        {
          REKut = P2zEG->getNeighbor(wGf2c++);
          if (0)
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x6e\x65\x69\x67\x68\x62\x6f\x72\x20\x6e\x6f\x2e\x20\x25\x64\x2c\x20\x70\x74\x72\x20\x3d\x20\x25\x70\x2c\x20\x69\x64\x20\x3d\x20\x25\x64"
                    "\n",
                    wGf2c, REKut, REKut ? REKut->mId : -1);
        }
        else
        {
          q73fi = false;
          REKut = reinterpret_cast<
              RoadHeader *>(mRoadData->findFirstNode(ODR_OPCODE_ROAD_HEADER));
        }
      }
      else
      {
        if (q73fi &&
            LOM4N.size())
        {
          q73fi = false;
          REKut = *gRJoU;
        }
        else if (LOM4N.size())
        {
          gRJoU++;
          if (gRJoU ==
              LOM4N.end())
            REKut = 0;
          else
            REKut = *gRJoU;
        }
        else
          REKut = reinterpret_cast<RoadHeader *>(
              REKut->getRight());
      }
      if (REKut)
        mHierPos.clearNode(ODR_OPCODE_ROAD_HEADER);
    }
    if (
        Y3vvh)
      fprintf(stderr,
              "\x4d\x4d\x4d\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x66\x6f\x75\x6e\x64\x4c\x61\x6e\x65\x20\x3d\x20\x25\x73"
              "\n",
              _HwNc ? "\x74\x72\x75\x65" : "\x66\x61\x6c\x73\x65");
    if (_HwNc && mAllowLaneTypeCar)
    {
      mHierPos.restore();
      if (mMinimizeDeltaZ)
        mOptimumSolution = false;
      return RESULT_ON_ROAD;
    }
    return RESULT_NOT_ON_ROAD;
  }
  int RoadQuery::inertial2geo(const Coord &Tlj7t, GeoCoord &V5tBk)
  {
    if (!checkForRoadData())
      return RESULT_ERROR;
    if (
        mRoadData->getProjection())
    {
      if (!(mRoadData->getProjection()->inertial2geo(Tlj7t,
                                                     V5tBk)))
        return RESULT_ON_ROAD;
    }
    std::cout<< "use this.";
    Header *hdr = mRoadData->getOdrHeader();
    if (!hdr)
      return RESULT_ERROR;
    V5tBk.setZ(Tlj7t.getZ() + hdr->mOrigHeight);
    V5tBk.setH(0.5 *
                   M_PI -
               Tlj7t.getH());
    V5tBk.setP(Tlj7t.getP());
    V5tBk.setR(Tlj7t.getR());
    V5tBk.setLat(hdr->mOrigLat + Tlj7t.getY() * GeoCoord::cDegPerM);
    double XKUvO = GeoCoord::
                       cDegPerM /
                   cos(hdr->mOrigLat / 180.0 * M_PI);
    V5tBk.setLong(hdr->mOrigLong + Tlj7t.getX() * XKUvO);
    return RESULT_ON_ROAD;
  }
  int RoadQuery::geo2inertial(const GeoCoord &V5tBk,
                              Coord &Tlj7t)
  {
    if (!checkForRoadData())
      return RESULT_ERROR;
    if (mRoadData->getProjection())
    {
      if (!(mRoadData->getProjection()->geo2inertial(V5tBk, Tlj7t)))
        return RESULT_ON_ROAD;
    }
    Header *hdr = mRoadData->getOdrHeader();
    if (!hdr)
      return RESULT_ERROR;
    Tlj7t.setZ(V5tBk.getZ() - hdr->mOrigHeight);
    Tlj7t.setH(0.5 * M_PI - V5tBk
                                .getH());
    Tlj7t.setP(V5tBk.getP());
    Tlj7t.setR(V5tBk.getR());
    Tlj7t.setY((V5tBk.getLat() - hdr->mOrigLat) / GeoCoord::cDegPerM);
    double XKUvO = GeoCoord::cDegPerM / cos(
                                            hdr->mOrigLat / 180.0 * M_PI);
    Tlj7t.setX((V5tBk.getLong() - hdr->mOrigLong) / XKUvO);
    return RESULT_ON_ROAD;
  }
  const double &RoadQuery::getMaxTrackPos() const { return mMaxTrackPos; }
  double RoadQuery::getCurvature() { return mCurvature; }
  bool RoadQuery ::getLaneCurvature(double &j6pBH, double &Kv87d, double &It5y8, double &KGtll)
  {
    j6pBH =
        mLaneCurvature;
    Kv87d = mLaneCurvatureDot;
    It5y8 = mLaneCurvatureVert;
    KGtll =
        mLaneCurvatureVertDot;
    return true;
  }
  double RoadQuery::getDeltaLaneDir() { return mLaneDeltaHdg; }
  double RoadQuery::getTrackWidth(const TrackCoord &HQGFE)
  {
    double
        jMAuF = 0.0;
    LaneSection *H4prs = track2laneSection(HQGFE);
    if (!H4prs)
    {
      return 0.0;
    }
    double rganP = HQGFE.getS() - H4prs->mS;
    Lane *lane = reinterpret_cast<Lane *>(H4prs->getChild());
    while (lane)
    {
      jMAuF += getLaneWidth(lane, rganP);
      lane = reinterpret_cast<
          Lane *>(lane->getRight());
    }
    return jMAuF;
  }
  int RoadQuery::getTrackHeading(const TrackCoord &HQGFE, double &jwOMF)
  {
    RoadHeader *REKut = trackId2Node(HQGFE);
    GeoHeader *
        xjOaT = track2geoHeader(REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
    if (!pWj2K)
    {
      std::cerr
          << "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x67\x65\x74\x54\x72\x61\x63\x6b\x48\x65\x61\x64\x69\x6e\x67\x3a\x20\x6d\x69\x73\x73\x69\x6e\x67\x20\x67\x65\x6f\x6d\x65\x74\x72\x79\x20\x6e\x6f\x64\x65\x2e"
          << std::endl;
      return RESULT_ERROR;
    }
    if (!pWj2K->s2h(HQGFE.getS(), jwOMF))
      return RESULT_ERROR;
    return RESULT_ON_ROAD;
  }
  int RoadQuery::getTrackAngles(const TrackCoord &HQGFE, Coord &svZro)
  {
    RoadHeader *REKut = trackId2Node(HQGFE);
    GeoHeader *
        xjOaT = track2geoHeader(REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
    double aOzQT;
    double
        hgSHv;
    if (getRoll(REKut, HQGFE, hgSHv) == RESULT_ON_ROAD)
      svZro.setR(hgSHv);
    if (!pWj2K
             ->s2h(HQGFE.getS(), aOzQT))
      return RESULT_ERROR;
    svZro.setH(aOzQT);
    double s4E1B;
    int
        result = getPitch(REKut, HQGFE, s4E1B);
    if (result == RESULT_ON_ROAD)
      svZro.setP(s4E1B);
    return result;
  }
  int RoadQuery::getTrackAnglesDot(const TrackCoord &HQGFE, Coord &
                                                                svZro)
  {
    RoadHeader *REKut = trackId2Node(HQGFE);
    GeoHeader *xjOaT = track2geoHeader(
        REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    GeoNode *pWj2K =
        reinterpret_cast<GeoNode *>(xjOaT->getChild());
    if (!pWj2K)
      return RESULT_ERROR;
    double hgSHv;
    if (getRollDot(REKut, HQGFE, hgSHv) == RESULT_ON_ROAD)
      svZro.setR(hgSHv);
    svZro.setH(pWj2K->s2curvature(HQGFE.getS()));
    double s4E1B;
    double G8lrl;
    int
        result = getPitchDot(REKut, HQGFE, s4E1B, G8lrl);
    if (result == RESULT_ON_ROAD)
    {
      svZro.setP(s4E1B);
      svZro.setZ(G8lrl);
    }
    return result;
  }
  int RoadQuery::getPitchAndZ(const TrackCoord &HQGFE, double &s4E1B, double &hkK5C)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    if (!wR6_y)
      return RESULT_ERROR;
    if (!Ekmkk(wR6_y, HQGFE, hkK5C))
    {
      double hgSHv = 0.0;
      getRoll(wR6_y, HQGFE, hgSHv);
      return st2pitchAndZ(wR6_y, HQGFE.getS(), HQGFE.getT(),
                          s4E1B, hkK5C, hgSHv);
    }
    else
    {
      s4E1B = 0.0;
      return RESULT_ON_ROAD;
    }
  }
  int RoadQuery::
      getPitch(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &s4E1B)
  {
    s4E1B = 0.0;
    if (!wR6_y)
      return RESULT_ERROR;
    Elevation *j6B6g = s2elevation(wR6_y, s2tolS(HQGFE.getS(),
                                                 wR6_y));
    if (!j6B6g)
      return RESULT_ON_ROAD;
    s4E1B = j6B6g->s2pitch(s2tolS(HQGFE.getS(), wR6_y));
    return RESULT_ON_ROAD;
  }
  int RoadQuery::getPitchDot(RoadHeader *wR6_y,
                             const TrackCoord &HQGFE, double &QkuuH, double &G8lrl)
  {
    QkuuH = 0.0;
    if (!wR6_y)
      return RESULT_ERROR;
    Elevation *j6B6g = s2elevation(wR6_y, s2tolS(HQGFE.getS(), wR6_y));
    if (!j6B6g)
      return RESULT_ON_ROAD;
    QkuuH = j6B6g->s2pitchDot(s2tolS(HQGFE.getS(), wR6_y));
    G8lrl = j6B6g->s2secondDeriv(s2tolS(HQGFE.getS(), wR6_y));
    return RESULT_ON_ROAD;
  }
  int RoadQuery::getPitchAndZAndPitchDot(RoadHeader *wR6_y, const TrackCoord &HQGFE,
                                         double &s4E1B, double &hkK5C, double &QkuuH, double &G8lrl)
  {
    QkuuH = 0.0;
    hkK5C = 0.0;
    s4E1B =
        0.0;
    G8lrl = 0.0;
    if (!wR6_y)
      return RESULT_ERROR;
    if (!Ekmkk(wR6_y, HQGFE, hkK5C))
    {
      Elevation *j6B6g = s2elevation(wR6_y, s2tolS(HQGFE.getS(), wR6_y));
      if (!j6B6g)
        return RESULT_ON_ROAD;
      hkK5C = j6B6g->s2z(s2tolS(HQGFE.getS(), wR6_y));
      LateralShape *qDMed =
          EBWwH(wR6_y, s2tolS(HQGFE.getS(), wR6_y), HQGFE.getT());
      if (qDMed)
      {
        double hgSHv = 0.0;
        getRoll(wR6_y, HQGFE, hgSHv);
        hkK5C += cos(hgSHv) * (qDMed->st2z(s2tolS(HQGFE.getS(),
                                                  wR6_y),
                                           HQGFE.getT()));
      }
      s4E1B = j6B6g->s2pitch(s2tolS(HQGFE.getS(), wR6_y));
      QkuuH =
          j6B6g->s2pitchDot(s2tolS(HQGFE.getS(), wR6_y));
      G8lrl = j6B6g->s2secondDeriv(s2tolS(
          HQGFE.getS(), wR6_y));
    }
    SurfaceCRG *oV99J = s2surface(wR6_y, s2tolS(HQGFE.getS(), wR6_y), SurfaceCRG::sPurposeElevation);
    if (oV99J)
    {
      if ((oV99J->mMode == oV99J->sModeAttached) || (oV99J->mMode == oV99J->sModeAttached0))
      {
        double Zbgoe = 0.0;
        double
            GbZcy = 0.0;
        oV99J->st2zp(s2tolS(HQGFE.getS(), wR6_y), HQGFE.getT(), Zbgoe, GbZcy, true);
        if (mUseContactPatch)
        {
          double QS1VX = s2tolS(HQGFE.getS(), wR6_y);
          double nM0cp = 0.0;
          double HCZvL = 0.0;
          oV99J->st2zp(QS1VX + 0.5 * mContactPatchLength, HQGFE.getT() - 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          oV99J->st2zp(
              QS1VX - 0.5 * mContactPatchLength, HQGFE.getT() - 0.5 * mContactPatchWidth, nM0cp, HCZvL,
              true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          oV99J->st2zp(QS1VX + 0.5 * mContactPatchLength, HQGFE.getT() + 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          oV99J
              ->st2zp(QS1VX - 0.5 * mContactPatchLength, HQGFE.getT() + 0.5 * mContactPatchWidth, nM0cp,
                      HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          Zbgoe /= 5.0;
          GbZcy /= 5.0;
        }
        Zbgoe *=
            mSurfaceScale;
        if (oV99J->mMode == oV99J->sModeAttached0)
          hkK5C = Zbgoe;
        else
          hkK5C +=
              Zbgoe;
      }
      else
      {
        Coord wiBPl;
        s4E1B = 0.0;
        QkuuH = 0.0;
        G8lrl = 0.0;
        if (track2inertialSimple(
                wiBPl, HQGFE, wR6_y) == RESULT_ON_ROAD)
        {
          oV99J->xy2zp(wiBPl.getX(), wiBPl.getY(), hkK5C, s4E1B, true);
          if (mUseContactPatch)
          {
            double dx = 0.5 * (mContactPatchLength * cos(wiBPl.getH()) - mContactPatchWidth * sin(wiBPl.getH()));
            double dy = 0.5 * (mContactPatchLength * sin(wiBPl.getH()) + mContactPatchWidth * cos(wiBPl.getH()));
            double nM0cp = 0.0;
            double
                HCZvL = 0.0;
            oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() + dy, nM0cp, HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() + dy, nM0cp, HCZvL,
                         true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() - dy,
                         nM0cp, HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() - dy, nM0cp, HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            hkK5C /= 5.0;
            s4E1B /= 5.0;
          }
          hkK5C *= mSurfaceScale;
        }
      }
    }
    return RESULT_ON_ROAD;
  }
  int RoadQuery::getRoll(const TrackCoord &HQGFE, double &hgSHv)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    return getRoll(wR6_y, HQGFE, hgSHv);
  }
  int RoadQuery::getRoll(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &hgSHv)
  {
    hgSHv = 0.0;
    Superelevation *DI_ZK = s2superelevation(
        wR6_y, s2tolS(HQGFE.getS(), wR6_y));
    if (DI_ZK)
      hgSHv = DI_ZK->s2xfall(s2tolS(HQGFE.getS(), wR6_y));
    return RESULT_ON_ROAD;
  }
  int RoadQuery::getRollDot(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &PwesK)
  {
    PwesK = 0.0;
    Superelevation *DI_ZK =
        s2superelevation(wR6_y, s2tolS(HQGFE.getS(), wR6_y));
    if (DI_ZK)
      PwesK = DI_ZK->s2xfallDot(s2tolS(HQGFE.getS(), wR6_y));
    return RESULT_ON_ROAD;
  }
  int RoadQuery::
      getRollAndRollDot(RoadHeader *wR6_y, const TrackCoord &HQGFE, double &hgSHv, double &PwesK)
  {
    PwesK = 0.0;
    hgSHv = 0.0;
    Superelevation *DI_ZK = s2superelevation(wR6_y, s2tolS(
                                                        HQGFE.getS(), wR6_y));
    if (DI_ZK)
    {
      hgSHv = DI_ZK->s2xfall(s2tolS(HQGFE.getS(), wR6_y));
      PwesK = DI_ZK->s2xfallDot(s2tolS(HQGFE.getS(), wR6_y));
    }
    return RESULT_ON_ROAD;
  }
  int RoadQuery::lane2track(LaneCoord &Hv9fs)
  {
    double rkiXc = 0.0;
    double lseyi = 0.0;
    double
        L9KFn = 0.0;
    double xPO_v = 0.0;
    double mng_B = 0.0;
    mLaneDeltaHdg = 0.0;
    LaneSection *H4prs =
        track2laneSection(Hv9fs, 0, true, Hv9fs.getLaneId());
    if (!H4prs)
    {
      return RESULT_ERROR;
    }
    double rganP = Hv9fs.getS() - H4prs->mS;
    Lane *lane = reinterpret_cast<Lane *>(H4prs->getChild());
    if (lane && Hv9fs.getLaneId() > lane->mId)
      return RESULT_NOT_ON_ROAD;
    rkiXc = track2laneOffset(Hv9fs, xPO_v, reinterpret_cast<RoadHeader *>(H4prs->getParent()));
    mLaneDeltaHdg = atan(xPO_v);
    mLaneCurvature = 0.0;
    while (lane)
    {
      if (lane->mId == Hv9fs.getLaneId())
      {
        mLaneWidth = getLaneWidth(lane, rganP, lseyi, L9KFn);
        mHierPos.registerNode(lane, ODR_OPCODE_LANE);
        lane2laneHeight(Hv9fs);
        if (lane->mId < 0)
        {
          rkiXc -= 0.5 * mLaneWidth;
          rkiXc += Hv9fs.getOffset();
          Hv9fs.setT(rkiXc);
          if (fabs(mLaneWidth) >
              1.0e-6)
          {
            mng_B += ((Hv9fs.getOffset() / mLaneWidth) + 0.5) * lseyi;
            mLaneCurvature -= ((
                                   Hv9fs.getOffset() / mLaneWidth) +
                               0.5) *
                              L9KFn;
          }
          mLaneDeltaHdg -= atan(mng_B);
          return RESULT_ON_ROAD;
        }
        rkiXc += 0.5 * mLaneWidth + Hv9fs.getOffset();
        if (fabs(mLaneWidth) >
            1.0e-6)
        {
          mng_B += ((Hv9fs.getOffset() / mLaneWidth) + 0.5) * lseyi;
          mLaneCurvature += ((
                                 Hv9fs.getOffset() / mLaneWidth) +
                             0.5) *
                            L9KFn;
        }
      }
      if (Hv9fs.getLaneId() >= 0)
      {
        if (lane->mId < Hv9fs.getLaneId())
        {
          rkiXc += getLaneWidth(lane, rganP, lseyi, L9KFn);
          mng_B += lseyi;
          mLaneCurvature += L9KFn;
        }
        if ((lane->mId == 0) || !(lane->getRight()))
        {
          Hv9fs.setT(rkiXc);
          mLaneDeltaHdg += atan(mng_B);
          return RESULT_ON_ROAD;
        }
      }
      else
      {
        if ((lane->mId <= 0) && (lane->mId > Hv9fs.getLaneId()))
        {
          rkiXc -= getLaneWidth(lane, rganP, lseyi, L9KFn);
          mng_B += lseyi;
          mLaneCurvature -= L9KFn;
        }
      }
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    mHierPos.registerNode(NULL, ODR_OPCODE_LANE);
    return RESULT_NOT_ON_ROAD;
  }
  int RoadQuery::track2lane(LaneCoord &Hv9fs, RoadHeader *M8Gux)
  {
    static bool PwJ1X = false;
    static bool H4Ubf = true;
    int amiY7 = 0;
    if (H4Ubf)
    {
      if (Hv9fs.getT() < -1.0e-6)
        amiY7 = -1;
      else if (Hv9fs.getT() > 1.0e-6)
        amiY7 = 1;
    }
    LaneSection *H4prs = track2laneSection(Hv9fs,
                                           M8Gux, H4Ubf, amiY7);
    mLaneCurvature = 0.0;
    mLaneDeltaHdg = 0.0;
    if (!H4prs)
    {
      if (PwJ1X)
        std ::cerr << "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x74\x72\x61\x63\x6b\x32\x6c\x61\x6e\x65\x3a\x20\x6e\x6f\x20\x6c\x61\x6e\x65\x20\x73\x65\x63\x74\x69\x6f\x6e\x20\x66\x6f\x75\x6e\x64\x21"
                   << std::endl;
      return RESULT_NOT_ON_ROAD;
    }
    else if (PwJ1X)
    {
      std::cerr << "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x74\x72\x61\x63\x6b\x32\x6c\x61\x6e\x65\x3a\x20\x6c\x61\x6e\x65\x20\x73\x65\x63\x74\x69\x6f\x6e\x20\x66\x6f\x75\x6e\x64\x21"
                << std::endl;
      H4prs->print(false);
    }
    double rganP = Hv9fs.getS() - H4prs->mS;
    Lane *lane =
        reinterpret_cast<Lane *>(H4prs->getChild());
    Lane *Mpq9o = 0;
    Lane *PuUTd = 0;
    while (lane)
    {
      if (lane->mId == 1)
        Mpq9o = lane;
      if (lane->mId == -1)
        PuUTd = lane;
      if (lane->mId == 0)
        break;
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    if (!lane)
    {
      if (H4prs->mSideMask !=
          H4prs->sLaneSecSideFlagAll)
      {
        if ((Hv9fs.getT() >= 0.0) && (H4prs->mSideMask & H4prs->sLaneSecSideFlagLeft))
          lane = Mpq9o;
        else if ((Hv9fs.getT() < 0.0) && (H4prs->mSideMask &
                                          H4prs->sLaneSecSideFlagRight))
          lane = PuUTd;
      }
      if (!lane)
      {
        return RESULT_NOT_ON_ROAD;
      }
    }
    double gV5Qq = Hv9fs.getT();
    double xPO_v = 0.0;
    gV5Qq -= track2laneOffset(Hv9fs, xPO_v,
                              M8Gux);
    mLaneDeltaHdg = atan(xPO_v);
    if (gV5Qq >= 0.0)
    {
      double qfK7D;
      double vLwwT = 0.0;
      double lseyi = 0.0;
      double L9KFn = 0.0;
      while (lane)
      {
        qfK7D = getLaneWidth(lane, rganP,
                             lseyi, L9KFn);
        if (vLwwT + qfK7D > gV5Qq)
        {
          vLwwT += 0.5 * qfK7D;
          double offset = gV5Qq - vLwwT;
          if (fabs(offset) < 1.0e-6)
            offset = 0.0;
          Hv9fs.setOffset(offset);
          Hv9fs.setLaneId(lane->mId);
          mLaneWidth = qfK7D;
          mLaneDeltaHdg += ((offset / qfK7D) + 0.5) * atan(lseyi);
          mHierPos.registerNode(lane, ODR_OPCODE_LANE);
          return RESULT_ON_ROAD;
        }
        vLwwT += qfK7D;
        mLaneDeltaHdg += atan(lseyi);
        lane = reinterpret_cast<Lane *>(lane->getLeft());
      }
      if (
          gV5Qq != 0.0)
      {
        mHierPos.registerNode(NULL, ODR_OPCODE_LANE);
        return RESULT_NOT_ON_ROAD;
      }
    }
    double qfK7D;
    double vLwwT = 0.0;
    double lseyi = 0.0;
    double L9KFn = 0.0;
    while (lane)
    {
      qfK7D = getLaneWidth(lane, rganP, lseyi, L9KFn);
      if (vLwwT + qfK7D > (-gV5Qq))
      {
        vLwwT += 0.5 * qfK7D;
        Hv9fs.setOffset(vLwwT + gV5Qq);
        Hv9fs.setLaneId(lane->mId);
        mLaneWidth = qfK7D;
        mHierPos.registerNode(lane, ODR_OPCODE_LANE);
        mLaneDeltaHdg += ((
                              Hv9fs.getOffset() / qfK7D) -
                          0.5) *
                         atan(lseyi);
        return RESULT_ON_ROAD;
      }
      vLwwT += qfK7D;
      mLaneDeltaHdg -= atan(lseyi);
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    mHierPos.registerNode(NULL, ODR_OPCODE_LANE);
    return RESULT_NOT_ON_ROAD;
  }
  double
  RoadQuery::track2laneOffset(const LaneCoord &Hv9fs, double &xPO_v, RoadHeader *M8Gux)
  {
    xPO_v = 0.0;
    if (!M8Gux)
      M8Gux = trackId2Node(Hv9fs);
    if (!M8Gux)
      return 0.0;
    LaneOffset *
        hYLzo = reinterpret_cast<LaneOffset *>(M8Gux->getChild(ODR_OPCODE_LANE_OFFSET));
    if (
        !hYLzo)
      return 0.0;
    while (hYLzo && (hYLzo->mSEnd < Hv9fs.getS()))
      hYLzo =
          reinterpret_cast<LaneOffset *>(hYLzo->getRight());
    if (!hYLzo)
      return 0.0;
    xPO_v =
        hYLzo->s2offsetDot(Hv9fs.getS());
    return hYLzo->s2offset(Hv9fs.getS());
  }
  int RoadQuery::track2curvature(const TrackCoord &HQGFE)
  {
    RoadHeader *REKut = trackId2Node(HQGFE);
    if (!REKut)
      return RESULT_ERROR;
    GeoHeader *xjOaT = track2geoHeader(REKut,
                                       HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    GeoNode *pWj2K = reinterpret_cast<
        GeoNode *>(xjOaT->getChild());
    mCurvature = geoNode2curvature(pWj2K, REKut, s2tolS(HQGFE.getS(), REKut));
    return RESULT_ON_ROAD;
  }
  int RoadQuery::lane2curvature(const LaneCoord &Hv9fs)
  {
    OpenDrive::LaneCoord RyKK9 = Hv9fs;
    mLaneCurvature = 0.0;
    track2curvature(RyKK9);
    if (fabs(mCurvature) > 1.e-5)
      mCurvature = 1.0 / (1.0 / mCurvature -
                          Hv9fs.getT());
    if (lane2track(RyKK9) != RESULT_ON_ROAD)
      return RESULT_NOT_ON_ROAD;
    double zJGTy = mLaneCurvature;
    double VB_b0 = mCurvature;
    double rganP = 0.0001;
    if (RyKK9
            .getS() > 2.0 * rganP)
      rganP *= -1.0;
    RyKK9.setS(RyKK9.getS() + rganP);
    if (lane2track(RyKK9) != RESULT_ON_ROAD)
      return RESULT_NOT_ON_ROAD;
    track2curvature(RyKK9);
    if (fabs(
            mCurvature) > 1.e-5)
      mCurvature = 1.0 / (1.0 / mCurvature - Hv9fs.getT());
    mLaneCurvatureDot = (mLaneCurvature + mCurvature - zJGTy - VB_b0) / rganP;
    mCurvature = VB_b0;
    mLaneCurvature =
        mCurvature + zJGTy;
    RoadHeader *wR6_y = trackId2Node(Hv9fs);
    double G8lrl;
    double S5i5l;
    if (getPitchDot(wR6_y, Hv9fs, mLaneCurvatureVert, G8lrl) != RESULT_ON_ROAD)
      return RESULT_NOT_ON_ROAD;
    RyKK9.setS(Hv9fs.getS() + rganP);
    if (getPitchDot(wR6_y, RyKK9,
                    S5i5l, G8lrl) != RESULT_ON_ROAD)
      return RESULT_NOT_ON_ROAD;
    mLaneCurvatureVertDot = (S5i5l - mLaneCurvatureVert) / rganP;
    return RESULT_ON_ROAD;
  }
  const double &RoadQuery::
      getLaneWidth() const { return mLaneWidth; }
  const double &RoadQuery::getLaneHeight()
      const
  {
    static const double HcKcu = 0.0;
    if (mUseLaneHeight)
      return mLaneHeight;
    return HcKcu;
  }
  int RoadQuery::lane2laneWidth(const LaneCoord &Hv9fs)
  {
    LaneSection *H4prs =
        track2laneSection(Hv9fs);
    if (!H4prs)
    {
      return RESULT_NOT_ON_ROAD;
    }
    double rganP =
        Hv9fs.getS() - H4prs->mS;
    Lane *lane = laneId2Node(Hv9fs.getLaneId());
    if (!lane)
    {
      lane =
          reinterpret_cast<Lane *>(H4prs->getChild());
      while (lane)
      {
        if (lane->mId == Hv9fs.getLaneId())
          break;
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
      if (!lane)
      {
        mHierPos.registerNode(lane, ODR_OPCODE_LANE);
        return RESULT_NOT_ON_ROAD;
      }
    }
    mLaneWidth = getLaneWidthAndRoadMark(lane, rganP);
    mHierPos.registerNode(lane,
                          ODR_OPCODE_LANE);
    return RESULT_ON_ROAD;
  }
  int RoadQuery::lane2roadMark(const LaneCoord &Hv9fs)
  {
    LaneSection *H4prs = track2laneSection(Hv9fs);
    if (!H4prs)
    {
      return RESULT_NOT_ON_ROAD;
    }
    double rganP = Hv9fs.getS() - H4prs->mS;
    Lane *lane = laneId2Node(
        Hv9fs.getLaneId());
    if (!lane)
    {
      lane = reinterpret_cast<Lane *>(H4prs->getChild());
      while (lane)
      {
        if (lane->mId == Hv9fs.getLaneId())
          break;
        lane = reinterpret_cast<Lane *>(
            lane->getRight());
      }
      if (!lane)
      {
        mHierPos.registerNode(lane, ODR_OPCODE_LANE);
        return RESULT_NOT_ON_ROAD;
      }
    }
    mRoadMark = getRoadMark(lane, rganP);
    mHierPos.registerNode(
        lane, ODR_OPCODE_LANE);
    return RESULT_ON_ROAD;
  }
  const unsigned short &RoadQuery::
      getRoadMark() const { return mRoadMark; }
  int RoadQuery::track2validTrack(TrackCoord &
                                      HQGFE,
                                  bool &dirChanged, bool SFH3M)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    if (!wR6_y)
      return RESULT_ERROR;
    if ((HQGFE.getS() >= (0.0 - mSTolerance)) && (HQGFE.getS() <= (wR6_y
                                                                       ->mLength +
                                                                   mSTolerance)))
      return RESULT_ON_ROAD;
    double rganP = 0.0;
    bool JN4Nc = false;
    double W_x8A = 0.0;
    if (HQGFE.getS() < 0.0)
    {
      rganP = -HQGFE.getS();
      JN4Nc = wR6_y->getPredecessorDir() == ODR_LINK_POINT_START;
      W_x8A = wR6_y->getPredecessorS();
      wR6_y =
          wR6_y->getPredecessor();
      if (wR6_y && JN4Nc)
        dirChanged = !dirChanged;
    }
    else
    {
      rganP = HQGFE
                  .getS() -
              wR6_y->mLength;
      JN4Nc = wR6_y->getSuccessorDir() == ODR_LINK_POINT_START;
      W_x8A = wR6_y->getSuccessorS();
      wR6_y = wR6_y->getSuccessor();
      if (wR6_y && !JN4Nc)
        dirChanged = !dirChanged;
    }
    if (!wR6_y)
      return RESULT_EXCEEDS_ROAD;
    HQGFE.setTrackId(
        wR6_y->mId);
    HQGFE.setTrackId(wR6_y->mIdAsString);
    if (SFH3M)
    {
      HQGFE.setS(W_x8A);
      if (
          dirChanged)
        HQGFE.setH(HQGFE.getH() + M_PI);
      if (JN4Nc)
        return addTrackS(HQGFE, rganP,
                         true);
      return addTrackS(HQGFE, -rganP, true);
    }
    if (JN4Nc)
      HQGFE.setS(W_x8A + rganP);
    else
      HQGFE.setS(W_x8A - rganP);
    if (dirChanged)
      HQGFE.setH(HQGFE.getH() + M_PI);
    return track2validTrack(HQGFE, dirChanged);
  }
  int RoadQuery::track2validTrack(TrackCoord &
                                      HQGFE)
  {
    bool dirChanged = false;
    return track2validTrack(HQGFE, dirChanged);
  }
  int RoadQuery::lane2validLane(LaneCoord &Hv9fs, bool &dirChanged)
  {
    RoadHeader *wR6_y =
        trackId2Node(Hv9fs);
    if (!wR6_y)
      return RESULT_ERROR;
    if (Hv9fs.getS() >= (0.0 -
                         mSTolerance) &&
        Hv9fs.getS() <= (wR6_y->mLength + mSTolerance))
      return RESULT_ON_ROAD;
    int pwxLH;
    double rganP = 0.0;
    bool JN4Nc = false;
    bool OjzDY = true;
    if (Hv9fs.getS() < 0.0)
    {
      rganP = -Hv9fs.getS();
      JN4Nc = wR6_y->getPredecessorDir() == ODR_LINK_POINT_START;
      pwxLH = getLaneOnPreviousRoad(Hv9fs, wR6_y, OjzDY);
      wR6_y = wR6_y->getPredecessor(true);
      if (JN4Nc)
        dirChanged = !dirChanged;
    }
    else
    {
      rganP = Hv9fs.getS() - wR6_y->mLength;
      JN4Nc =
          wR6_y->getSuccessorDir() == ODR_LINK_POINT_START;
      pwxLH = getLaneOnNextRoad(Hv9fs,
                                wR6_y, OjzDY);
      wR6_y = wR6_y->getSuccessor(true);
      if (!JN4Nc)
        dirChanged = !dirChanged;
    }
    if (!wR6_y || !OjzDY)
      return RESULT_EXCEEDS_ROAD;
    if (0 && (wR6_y->mJuncNo >= 0))
    {
      fprintf(
          stderr,
          "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x6c\x61\x6e\x65\x32\x76\x61\x6c\x69\x64\x4c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x6a\x75\x6e\x63\x74\x69\x6f\x6e\x20\x70\x61\x74\x68\x20\x25\x64\x2c\x20\x6f\x6c\x64\x4c\x61\x6e\x65\x20\x3d\x20\x25\x64\x2c\x20\x6e\x65\x77\x4c\x61\x6e\x65\x20\x3d\x20\x25\x64"
          "\n",
          wR6_y->mId, Hv9fs.getLaneId(), pwxLH);
    }
    Hv9fs.setTrackId(wR6_y->mId);
    Hv9fs.setTrackId(wR6_y->mIdAsString);
    Hv9fs.setLaneId(pwxLH);
    if (JN4Nc)
      Hv9fs.setS(rganP);
    else
      Hv9fs.setS(wR6_y->mLength - rganP);
    Coord wpcvC;
    getTrackAngles(Hv9fs, wpcvC);
    Hv9fs.setH(wpcvC.getH());
    return lane2validLane(Hv9fs, dirChanged);
  }
  int RoadQuery ::lane2validLane(LaneCoord &Hv9fs)
  {
    bool dirChanged = false;
    return lane2validLane(
        Hv9fs, dirChanged);
  }
  bool RoadQuery::getCrossSection(const TrackCoord &HQGFE,
                                  LaneInfVec &cxBiL)
  {
    LaneSection *H4prs = track2laneSection(HQGFE);
    if (!H4prs)
    {
      return false;
    }
    double rganP = HQGFE.getS() - H4prs->mS;
    Lane *lane = reinterpret_cast<Lane *>(
        H4prs->getChild());
    while (lane)
    {
      struct LaneInfoStruct qzPLJ;
      qzPLJ.width =
          getLaneWidth(lane, rganP);
      qzPLJ.id = lane->mId;
      qzPLJ.type = lane->mType;
      qzPLJ.lane =
          lane;
      cxBiL.push_back(qzPLJ);
      lane = reinterpret_cast<Lane *>(lane->getRight());
    }
    return true;
  }
  LaneSection *RoadQuery::track2laneSection(const TrackCoord &HQGFE,
                                            RoadHeader *M8Gux, bool checkSide, const int &laneId)
  {
    RoadHeader *wR6_y = M8Gux;
    RoadHeader *f5FQ2 = reinterpret_cast<RoadHeader *>(mHierPos.findNode(
        ODR_OPCODE_ROAD_HEADER));
    if (!wR6_y)
      wR6_y = trackId2Node(HQGFE);
    else
    {
      bool FulB_ = (wR6_y->mId == (unsigned int)HQGFE.getTrackId());
      if (HQGFE.hasStringId())
        FulB_ = (wR6_y->mIdAsString == HQGFE.getTrackIdAsString());
      if (!FulB_)
        wR6_y = trackId2Node(
            HQGFE);
      else if (wR6_y != f5FQ2)
        mHierPos.registerNode(wR6_y, ODR_OPCODE_ROAD_HEADER);
    }
    if (!wR6_y)
      return 0;
    if (HQGFE.getS() < (0.0 - mSTolerance) || HQGFE.getS() > (wR6_y->mLength + mSTolerance))
    {
      return 0;
    }
    LaneSection *sec = reinterpret_cast<LaneSection *>(
        mHierPos.findNode(ODR_OPCODE_LANE_SECTION));
    double DFg6k = HQGFE.getS();
    if (sec && ((
                    sec->mS <= DFg6k) &&
                (sec->mSEnd >= DFg6k) && (!checkSide || sec->containsSide(laneId))))
    {
      return sec;
    }
    if (checkSide)
    {
      sec = wR6_y->getFirstLaneSection();
      while (sec)
      {
        if ((sec->mS <= DFg6k) && (sec->mSEnd >= DFg6k) && sec->containsSide(laneId))
          break;
        sec =
            reinterpret_cast<LaneSection *>(sec->getRight());
      }
      mHierPos.registerNode(sec,
                            ODR_OPCODE_LANE_SECTION);
      return sec;
    }
    if (!sec)
      sec = wR6_y->getFirstLaneSection();
    if (sec)
    {
      if (sec->mS > DFg6k)
      {
        while (sec && sec->mS > DFg6k)
          sec = reinterpret_cast<
              LaneSection *>(sec->getLeft());
      }
      else
      {
        while (sec && sec->mSEnd < DFg6k)
          sec =
              reinterpret_cast<LaneSection *>(sec->getRight());
      }
    }
    mHierPos.registerNode(sec,
                          ODR_OPCODE_LANE_SECTION);
    return sec;
  }
  int RoadQuery::track2inertialAngDotCrossSec(const TrackCoord &HQGFE, Coord &Tlj7t, Coord &oUumu, LaneInfVec &cxBiL)
  {
    mMaxTrackPos =
        0.0;
    RoadHeader *REKut = trackId2Node(HQGFE);
    if (!REKut)
      return RESULT_ERROR;
    GeoHeader
        *xjOaT = track2geoHeader(REKut, HQGFE);
    if (!xjOaT)
    {
      return RESULT_EXCEEDS_ROAD;
    }
    mMaxTrackPos = REKut->mLength;
    GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
    double x;
    double y;
    double aOzQT;
    double rkiXc = HQGFE.getT();
    double
        hgSHv;
    double PwesK;
    if (getRollAndRollDot(REKut, HQGFE, hgSHv, PwesK) == RESULT_ON_ROAD)
    {
      rkiXc *= cos(hgSHv);
      Tlj7t.setR(hgSHv);
      oUumu.setR(PwesK);
    }
    if (!pWj2K->st2xyh(HQGFE
                           .getS(),
                       rkiXc, x, y, aOzQT))
      return RESULT_ERROR;
    Tlj7t.setX(x);
    Tlj7t.setY(y);
    Tlj7t.setH(aOzQT);
    mCurvature = geoNode2curvature(pWj2K, REKut, HQGFE.getS());
    oUumu.setH(
        mCurvature);
    double s4E1B;
    double hkK5C;
    double QkuuH;
    double G8lrl;
    int result;
    if ((
            result = getPitchAndZAndPitchDot(REKut, HQGFE, s4E1B, hkK5C, QkuuH, G8lrl)) !=
        RESULT_ON_ROAD)
      return result;
    Tlj7t.setP(s4E1B);
    hkK5C += HQGFE.getT() * sin(Tlj7t.getR());
    Tlj7t.setZ(hkK5C);
    oUumu.setP(QkuuH);
    oUumu.setZ(G8lrl);
    getCrossSection(
        HQGFE, cxBiL);
    return RESULT_ON_ROAD;
  }
  RoadHeader *RoadQuery::trackId2Node(const int
                                          id) { return trackId2Node((const unsigned int)(id)); }
  RoadHeader *RoadQuery::
      trackId2Node(const unsigned int id)
  {
    if (!checkForRoadData())
      return NULL;
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(mHierPos.findNode(
        ODR_OPCODE_ROAD_HEADER));
    if (REKut && REKut->mId == id)
      return REKut;
    REKut =
        reinterpret_cast<RoadHeader *>(mRoadData->getNodeFromId(ODR_OPCODE_ROAD_HEADER, id));
    mHierPos.registerNode(REKut, ODR_OPCODE_ROAD_HEADER);
    return REKut;
  }
  RoadHeader *
  RoadQuery::trackId2Node(const std::string &id)
  {
    if (!checkForRoadData())
      return NULL;
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(mHierPos.findNode(
        ODR_OPCODE_ROAD_HEADER));
    if (REKut && REKut->mIdAsString == id)
      return REKut;
    REKut =
        reinterpret_cast<RoadHeader *>(mRoadData->getNodeFromId(ODR_OPCODE_ROAD_HEADER, id));
    mHierPos.registerNode(REKut, ODR_OPCODE_ROAD_HEADER);
    return REKut;
  }
  RoadHeader *
  RoadQuery::trackId2Node(const TrackCoord &HQGFE)
  {
    if (HQGFE.hasStringId())
      return trackId2Node(HQGFE.getTrackIdAsString());
    return trackId2Node(HQGFE.getTrackId());
  }
  bool RoadQuery::getSignals(const TrackCoord &HQGFE, bool JN4Nc, const double &F5cuE, ResultVec &JWpF7)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    if (!wR6_y)
      return false;
    Signal *Pq1dJ;
    SignalRef *aMxCB;
    double X30nD = 0.0;
    double E5sRl = HQGFE.getS();
    bool xGrdy = false;
    while (wR6_y && (X30nD < F5cuE))
    {
      if (JN4Nc)
        Pq1dJ = reinterpret_cast<
            Signal *>(wR6_y->getChild(ODR_OPCODE_SIGNAL));
      else
        Pq1dJ = reinterpret_cast<Signal *>(wR6_y->getLastChild(ODR_OPCODE_SIGNAL));
      while (Pq1dJ && !xGrdy)
      {
        bool M4B1O = false;
        if (JN4Nc)
          M4B1O = Pq1dJ->mType >= 0 && Pq1dJ->mS >= (E5sRl - mSTolerance) && (Pq1dJ->mDir != ODR_DIRECTION_MINUS);
        else
          M4B1O = Pq1dJ->mType >= 0 && Pq1dJ->mS <= (E5sRl + mSTolerance) && (Pq1dJ->mDir != ODR_DIRECTION_PLUS);
        if (M4B1O)
        {
          struct ResultInfoStruct B8_rR;
          B8_rR.node = Pq1dJ;
          B8_rR.dist = X30nD + (JN4Nc ? 1.0 : -1.0) * (Pq1dJ->mS - E5sRl);
          if (B8_rR.dist > F5cuE)
            xGrdy = true;
          else
            JWpF7.push_back(B8_rR);
        }
        Pq1dJ = reinterpret_cast<Signal
                                     *>(JN4Nc ? Pq1dJ->getRight() : Pq1dJ->getLeft());
      }
      if (JN4Nc)
        aMxCB = reinterpret_cast<
            SignalRef *>(wR6_y->getChild(ODR_OPCODE_SIGNAL_REFERENCE));
      else
        aMxCB =
            reinterpret_cast<SignalRef *>(wR6_y->getLastChild(ODR_OPCODE_SIGNAL_REFERENCE));
      while (aMxCB)
      {
        Pq1dJ = aMxCB->mSignal;
        if (Pq1dJ)
        {
          bool M4B1O = false;
          if (JN4Nc)
            M4B1O =
                Pq1dJ->mType >= 0 && aMxCB->mS >= (E5sRl - mSTolerance) && (aMxCB->mDir != ODR_DIRECTION_MINUS);
          else
            M4B1O = Pq1dJ->mType >= 0 && aMxCB->mS <= (E5sRl + mSTolerance) && (aMxCB->mDir == ODR_DIRECTION_PLUS);
          if (M4B1O)
          {
            struct ResultInfoStruct B8_rR;
            B8_rR.node = Pq1dJ;
            B8_rR.dist = X30nD + (JN4Nc ? 1.0 : -1.0) * (aMxCB->mS - E5sRl);
            if (B8_rR.dist > F5cuE)
              return !JWpF7.empty();
            JWpF7.push_back(B8_rR);
          }
        }
        aMxCB = reinterpret_cast<
            SignalRef *>(JN4Nc ? aMxCB->getRight() : aMxCB->getLeft());
      }
      if (xGrdy)
        return !JWpF7.empty();
      if (JN4Nc)
      {
        X30nD += wR6_y->mLength - E5sRl;
        JN4Nc = wR6_y->getSuccessorDir() ==
                ODR_LINK_POINT_START;
        wR6_y = wR6_y->getSuccessor(true);
      }
      else
      {
        X30nD += E5sRl;
        JN4Nc =
            wR6_y->getPredecessorDir() == ODR_LINK_POINT_START;
        wR6_y = wR6_y->getPredecessor(
            true);
      }
      if (wR6_y)
        E5sRl = JN4Nc ? 0.0 : wR6_y->mLength;
    }
    return !JWpF7.empty();
  }
  bool
  RoadQuery::getObjects(const TrackCoord &HQGFE, bool JN4Nc, const double &F5cuE,
                        ResultVec &JWpF7)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    if (!wR6_y)
      return false;
    Object *U1jAq;
    double X30nD = 0.0;
    double E5sRl = HQGFE.getS();
    bool xGrdy = false;
    while (
        wR6_y && (X30nD < F5cuE))
    {
      if (JN4Nc)
        U1jAq = reinterpret_cast<Object *>(wR6_y->getChild(
            ODR_OPCODE_OBJECT));
      else
        U1jAq = reinterpret_cast<Object *>(wR6_y->getLastChild(
            ODR_OPCODE_OBJECT));
      while (U1jAq && !xGrdy)
      {
        bool FV43j = false;
        if (JN4Nc)
          FV43j = (U1jAq
                       ->mS >= (E5sRl - mSTolerance)) &&
                  (U1jAq->mDir != ODR_DIRECTION_MINUS);
        else
          FV43j = (U1jAq
                       ->mS <= (E5sRl + mSTolerance)) &&
                  (U1jAq->mDir != ODR_DIRECTION_PLUS);
        if (FV43j)
        {
          struct
              ResultInfoStruct B8_rR;
          B8_rR.node = U1jAq;
          B8_rR.dist = X30nD + (JN4Nc ? 1.0 : -1.0) * (U1jAq
                                                           ->mS -
                                                       E5sRl);
          if (B8_rR.dist > F5cuE)
            xGrdy = true;
          else
            JWpF7.push_back(B8_rR);
        }
        U1jAq =
            reinterpret_cast<Object *>(JN4Nc ? U1jAq->getRight() : U1jAq->getLeft());
      }
      if (xGrdy)
        return !JWpF7.empty();
      if (JN4Nc)
      {
        X30nD += wR6_y->mLength - E5sRl;
        JN4Nc = wR6_y->getSuccessorDir() == ODR_LINK_POINT_START;
        wR6_y = wR6_y->getSuccessor(true);
      }
      else
      {
        X30nD += E5sRl;
        JN4Nc = wR6_y->getPredecessorDir() == ODR_LINK_POINT_START;
        wR6_y = wR6_y
                    ->getPredecessor(true);
      }
      if (wR6_y)
        E5sRl = JN4Nc ? 0.0 : wR6_y->mLength;
    }
    return !JWpF7.empty();
  }
  const Coord &RoadQuery::getFootPoint() const { return mFootPoint; }
  Node *
  RoadQuery::getQueriedNodeOfType(const unsigned int type) { return mHierPos.findNode(type); }
  bool RoadQuery::getMaterial(const LaneCoord &Hv9fs, unsigned int &code, double &jGi0_, double &zJQyy, std::string &o055c)
  {
    bool fUhSQ = false;
    code = 0;
    jGi0_ =
        1.0;
    zJQyy = 0.0;
    o055c = std::string("");
    LaneSection *sec = reinterpret_cast<LaneSection
                                            *>(mHierPos.findNode(ODR_OPCODE_LANE_SECTION));
    if (!sec)
      return false;
    RoadHeader *
        M8Gux = reinterpret_cast<RoadHeader *>(sec->getParent());
    if (M8Gux && (mSurfaceScale !=
                  0.0))
    {
      SurfaceCRG *oV99J = M8Gux->s2surface(Hv9fs.getS(), SurfaceCRG::
                                                             sPurposeFriction);
      if (oV99J)
      {
        if ((oV99J->mMode == oV99J->sModeAttached) || (oV99J->mMode == oV99J->sModeAttached0))
        {
          double Zbgoe = 0.0;
          double GbZcy = 0.0;
          oV99J->st2zp(
              Hv9fs.getS(), Hv9fs.getT(), Zbgoe, GbZcy, false);
          if (mUseContactPatch)
          {
            double nM0cp =
                0.0;
            double HCZvL = 0.0;
            oV99J->st2zp(Hv9fs.getS() + 0.5 * mContactPatchLength, Hv9fs.getT() - 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
            Zbgoe += nM0cp;
            GbZcy += HCZvL;
            oV99J
                ->st2zp(Hv9fs.getS() - 0.5 * mContactPatchLength, Hv9fs.getT() - 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
            Zbgoe += nM0cp;
            GbZcy += HCZvL;
            oV99J->st2zp(Hv9fs.getS() + 0.5 *
                                            mContactPatchLength,
                         Hv9fs.getT() + 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
            Zbgoe += nM0cp;
            GbZcy += HCZvL;
            oV99J->st2zp(Hv9fs.getS() - 0.5 * mContactPatchLength, Hv9fs.getT() + 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
            Zbgoe += nM0cp;
            GbZcy += HCZvL;
            Zbgoe /= 5.0;
            GbZcy /= 5.0;
          }
          jGi0_ = Zbgoe * mSurfaceScale;
          fUhSQ = true;
        }
        else
        {
          Coord wiBPl;
          double
              Zbgoe = 0.0;
          double GbZcy = 0.0;
          TrackCoord HQGFE(M8Gux->mId, Hv9fs.getS(), Hv9fs.getT());
          HQGFE.setTrackId(M8Gux->mIdAsString);
          if (track2inertialSimple(wiBPl, HQGFE, M8Gux) == RESULT_ON_ROAD)
          {
            oV99J->xy2zp(wiBPl.getX(), wiBPl.getY(), Zbgoe, GbZcy, false);
            if (
                mUseContactPatch)
            {
              double dx = 0.5 * (mContactPatchLength * cos(wiBPl.getH()) -
                                 mContactPatchWidth * sin(wiBPl.getH()));
              double dy = 0.5 * (mContactPatchLength * sin(
                                                           wiBPl.getH()) +
                                 mContactPatchWidth * cos(wiBPl.getH()));
              double nM0cp = 0.0;
              double
                  HCZvL = 0.0;
              oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() + dy, nM0cp, HCZvL, true);
              Zbgoe +=
                  nM0cp;
              GbZcy += HCZvL;
              oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() + dy, nM0cp, HCZvL, true);
              Zbgoe += nM0cp;
              GbZcy += HCZvL;
              oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() - dy, nM0cp,
                           HCZvL, true);
              Zbgoe += nM0cp;
              GbZcy += HCZvL;
              oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() - dy, nM0cp, HCZvL, true);
              Zbgoe += nM0cp;
              GbZcy += HCZvL;
              Zbgoe /= 5.0;
              GbZcy /= 5.0;
            }
            jGi0_ =
                Zbgoe * mSurfaceScale;
            fUhSQ = true;
          }
        }
      }
    }
    Lane *lane = reinterpret_cast<Lane *>(mHierPos.findNode(ODR_OPCODE_LANE));
    if (!lane)
      return fUhSQ;
    double rganP = Hv9fs.getS() - sec->mS;
    LaneMaterial *Ac9rc;
    LaneMaterial *R4NFQ = reinterpret_cast<LaneMaterial *>(lane->getChild(ODR_OPCODE_LANE_MATERIAL));
    if (!R4NFQ)
      return fUhSQ;
    if (R4NFQ->mOffset >
        rganP)
      return fUhSQ;
    do
    {
      Ac9rc = R4NFQ;
      R4NFQ = reinterpret_cast<LaneMaterial *>(Ac9rc->getRight());
    } while (R4NFQ && R4NFQ->mOffset < rganP);
    code = Ac9rc->mCode;
    zJQyy = Ac9rc->mRoughness;
    o055c = Ac9rc->mSurface;
    if (!fUhSQ)
    {
      jGi0_ = Ac9rc->mFriction;
      if ((jGi0_ < 0.0) && (mLaneWidth > 0.0))
      {
        unsigned int YLKy_ = (unsigned int)(fabs(Ac9rc->mFriction));
        double FBN5q = 0.01 * (YLKy_ / 10000);
        double x5OAp = 0.01 * (YLKy_ % 10000);
        double offset =
            Hv9fs.getOffset() + 0.5 * mLaneWidth;
        if (Hv9fs.getLaneId() < 0)
          jGi0_ = x5OAp + (FBN5q - x5OAp) / mLaneWidth * offset;
        else
          jGi0_ = FBN5q + (x5OAp - FBN5q) / mLaneWidth * offset;
      }
    }
    return true;
  }
  bool RoadQuery::intersectCircle(const Coord &wiBPl, const double &tyz2F,
                                  LaneCoord::LaneVec &D7gpt)
  {
    D7gpt.clear();
    double x4ov2 = tyz2F * tyz2F;
    RoadHeader *
        REKut = reinterpret_cast<RoadHeader *>(mRoadData->findFirstNode(
            ODR_OPCODE_ROAD_HEADER));
    while (REKut)
    {
      GeoHeader *xjOaT = REKut->getFirstGeoHeader();
      bool CYug9 = false;
      while (xjOaT && !CYug9)
      {
        double dx = xjOaT->mX - wiBPl.getX();
        double
            dy = xjOaT->mY - wiBPl.getY();
        double yDDyk = (dx * dx + dy * dy);
        bool h1Z4_ = yDDyk < x4ov2;
        dx =
            xjOaT->mXEnd - wiBPl.getX();
        dy = xjOaT->mYEnd - wiBPl.getY();
        double aRCnf = (dx * dx + dy * dy);
        bool KLpKT = aRCnf < x4ov2;
        if (h1Z4_ != KLpKT)
        {
          dx = xjOaT->mXCtr - wiBPl.getX();
          dy = xjOaT
                   ->mYCtr -
               wiBPl.getY();
          double JjmY0 = (dx * dx + dy * dy);
          bool apGDj = JjmY0 < x4ov2;
          double
              YB9Rl = h1Z4_ ? xjOaT->mS : xjOaT->mSEnd;
          if (apGDj)
          {
            double ugEJK = h1Z4_ ? fabs(aRCnf - x4ov2) : fabs(yDDyk - x4ov2);
            double FbD7g = x4ov2 - JjmY0;
            double xofsV = FbD7g / (ugEJK + FbD7g);
            if (xofsV > 1.0)
              xofsV = 1.0;
            YB9Rl = 0.5 * (xjOaT->mSEnd + xjOaT->mS) + ((h1Z4_ ? 1.0 : -1.0) * 0.5 *
                                                        xjOaT->mLength * xofsV);
          }
          else if (xjOaT->inBoundingBox(wiBPl.getX(), wiBPl.getY()))
          {
            GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
            if (pWj2K)
            {
              double
                  rz9Cj;
              double rkiXc;
              if (pWj2K->xy2st(wiBPl.getX(), wiBPl.getY(), rz9Cj, rkiXc))
              {
                if (
                    h1Z4_)
                  YB9Rl = rz9Cj + tyz2F;
                else
                  YB9Rl = rz9Cj - tyz2F;
              }
            }
          }
          else
          {
            if (h1Z4_)
              YB9Rl = xjOaT->mS +
                      tyz2F - sqrt(yDDyk);
            else
              YB9Rl = xjOaT->mSEnd - (tyz2F - sqrt(aRCnf));
          }
          YB9Rl = std::min(
              YB9Rl, xjOaT->mSEnd);
          YB9Rl = std::max(YB9Rl, xjOaT->mS);
          LaneCoord Hv9fs(REKut->mId, 0, YB9Rl);
          Hv9fs.setTrackId(REKut->mIdAsString);
          if (h1Z4_)
            Hv9fs.setH(M_PI);
          D7gpt.push_back(Hv9fs);
        }
        else if (xjOaT->mLength > 3.0 * tyz2F)
        {
          if (xjOaT->inBoundingBox(
                  wiBPl.getX(), wiBPl.getY()))
          {
            GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
            if (pWj2K)
            {
              double rz9Cj;
              double rkiXc;
              if (pWj2K->xy2st(wiBPl.getX(),
                               wiBPl.getY(), rz9Cj, rkiXc))
              {
                double YB9Rl = std::min(rz9Cj + tyz2F, xjOaT->mSEnd);
                LaneCoord Hv9fs(REKut->mId, 0, YB9Rl);
                Hv9fs.setTrackId(REKut->mIdAsString);
                Hv9fs.setH(M_PI);
                D7gpt.push_back(Hv9fs);
                YB9Rl = std::max(rz9Cj - tyz2F, xjOaT->mS);
                Hv9fs.setS(YB9Rl);
                Hv9fs.setH(0.0);
                D7gpt.push_back(Hv9fs);
              }
            }
          }
        }
        xjOaT = reinterpret_cast<
            GeoHeader *>(xjOaT->getRight());
      }
      REKut = reinterpret_cast<RoadHeader *>(REKut->getRight());
    }
    return !D7gpt.empty();
  }
  void RoadQuery::setTolerance(const double &
                                   FfN9a) { mSTolerance = FfN9a; }

  bool RoadQuery::getLaneSpeed(const LaneCoord &Hv9fs,
                               double &kMjGG)
  {
    LaneSection *sec = reinterpret_cast<LaneSection *>(mHierPos.findNode(
        ODR_OPCODE_LANE_SECTION));
    if (!sec)
    {
      LaneCoord wxjcS = Hv9fs;
      if (lane2track(wxjcS) !=
          RESULT_ON_ROAD)
        return false;
    }
    sec = reinterpret_cast<LaneSection *>(mHierPos.findNode(ODR_OPCODE_LANE_SECTION));
    if (!sec)
      return false;
    Lane *lane =
        reinterpret_cast<Lane *>(mHierPos.findNode(ODR_OPCODE_LANE));
    if (!lane)
      return false;
    double rganP = Hv9fs.getS() - sec->mS;
    LaneSpeed *ayRBE;
    LaneSpeed *INpUf =
        reinterpret_cast<LaneSpeed *>(lane->getChild(ODR_OPCODE_LANE_SPEED));
    if (!INpUf)
      return laneSpeedFromRoadType(Hv9fs, kMjGG);
    if (INpUf->mOffset > rganP)
      return false;
    do
    {
      ayRBE = INpUf;
      INpUf = reinterpret_cast<LaneSpeed *>(ayRBE->getRight());
    } while (
        INpUf && INpUf->mOffset < rganP);
    kMjGG = ayRBE->mSpeed;
    return true;
  }
  int RoadQuery::
      getRoadType(const LaneCoord &Hv9fs)
  {
    mHierPos.clearNode(ODR_OPCODE_ROAD_TYPE);
    RoadHeader *REKut = reinterpret_cast<RoadHeader *>(mHierPos.findNode(
        ODR_OPCODE_ROAD_HEADER));
    if (!REKut)
      return false;
    RoadType *gxiSn = REKut->getFirstRoadType();
    if (!gxiSn)
      return ODR_ROAD_TYPE_NONE;
    while (gxiSn)
    {
      if ((gxiSn->mStartPos <= Hv9fs.getS()) && (gxiSn->mEndPos >= Hv9fs.getS()))
      {
        mHierPos.registerNode(
            gxiSn, ODR_OPCODE_ROAD_TYPE);
        return gxiSn->mType;
      }
      gxiSn = reinterpret_cast<RoadType
                                   *>(gxiSn->getRight());
    }
    return ODR_ROAD_TYPE_NONE;
  }
  bool RoadQuery::
      collectJuncInfo(JuncHeader *jHdr, JuncInfVec &JWpF7, RoadHeader *inRoad)
  {
    JWpF7.clear();
    if (!jHdr)
      return false;
    JuncLink *hYfdO = reinterpret_cast<JuncLink *>(jHdr->getChild(ODR_OPCODE_JUNCTION_LINK));
    while (hYfdO)
    {
      if (!inRoad || (inRoad == hYfdO->mIncomingRoad))
      {
        JunctionInfoStruct j7wz6;
        j7wz6.jHdr = jHdr;
        j7wz6.inRoad = hYfdO->mIncomingRoad;
        j7wz6.connRoad = hYfdO->mConnectingRoad;
        j7wz6.turnAngle = hYfdO->mTurnAngle;
        JuncLaneLink *SJBOF = hYfdO->getFirstLaneLink();
        while (SJBOF)
        {
          if (SJBOF ==
              hYfdO->getFirstLaneLink())
          {
            j7wz6.minLaneIn = SJBOF->mRoadLane;
            j7wz6.maxLaneIn =
                j7wz6.minLaneIn;
          }
          else
          {
            if (SJBOF->mRoadLane < j7wz6.minLaneIn)
              j7wz6.minLaneIn = SJBOF
                                    ->mRoadLane;
            if (SJBOF->mRoadLane > j7wz6.maxLaneIn)
              j7wz6.maxLaneIn = SJBOF->mRoadLane;
          }
          SJBOF = reinterpret_cast<JuncLaneLink *>(SJBOF->getRight());
        }
        JWpF7.push_back(
            j7wz6);
      }
      hYfdO = reinterpret_cast<JuncLink *>(hYfdO->getRight());
    }
    return JWpF7.size() != 0;
  }
  bool RoadQuery::getNextJunction(const TrackCoord &HQGFE, bool JN4Nc,
                                  JuncHeader *&jHdr, double &ioAAo, RoadHeader *&inRoad)
  {
    jHdr = 0;
    inRoad = 0;
    ioAAo = 0.0;
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    if (!wR6_y)
      return false;
    int McoAR = 0;
    RoadHeader *RfPqw = wR6_y;
    double E5sRl = HQGFE.getS();
    while (wR6_y && (McoAR++ < 50))
    {
      if (
          JN4Nc)
      {
        ioAAo += wR6_y->mLength - E5sRl;
        if (wR6_y->getSuccessor())
        {
          JN4Nc = wR6_y->getSuccessorDir() == ODR_LINK_POINT_START;
          wR6_y = wR6_y->getSuccessor();
        }
        else if (
            wR6_y->getSuccessorNode() && (wR6_y->getSuccessorNode()->getOpcode() ==
                                          ODR_OPCODE_JUNCTION_HEADER))
        {
          jHdr = reinterpret_cast<JuncHeader *>(wR6_y->getSuccessorNode());
          inRoad = wR6_y;
          return true;
        }
        else
          return false;
      }
      else
      {
        ioAAo +=
            E5sRl;
        if (wR6_y->getPredecessor())
        {
          JN4Nc = wR6_y->getPredecessorDir() ==
                  ODR_LINK_POINT_START;
          wR6_y = wR6_y->getPredecessor();
        }
        else if (wR6_y->getPredecessorNode() && (wR6_y->getPredecessorNode()->getOpcode() ==
                                                 ODR_OPCODE_JUNCTION_HEADER))
        {
          jHdr = reinterpret_cast<JuncHeader *>(wR6_y->getPredecessorNode());
          inRoad = wR6_y;
          return true;
        }
        else
          return false;
      }
      if (wR6_y)
      {
        if (
            wR6_y == RfPqw)
          return false;
        E5sRl = JN4Nc ? 0.0 : wR6_y->mLength;
      }
    }
    return false;
  }
  void
  RoadQuery::setRoadData(RoadData *data)
  {
    mRoadData = data;
    mExplicitRoadData = mRoadData != 0;
  }
  void RoadQuery::setDebugMask(unsigned int GhT88) { mDebugMask = GhT88; }
  void
  RoadQuery::setMaxTrackDist(const double &F5cuE) { mMaxTrackDist = F5cuE; }
  void
  RoadQuery::clearHistory() { mHierPos.registerNode(0, ODR_OPCODE_ROAD_HEADER); }
  void
  RoadQuery::fillHistory(const HierCoord &SkShA) { mHierPos = SkShA; }
  const HierCoord &
  RoadQuery::getHistory() const { return mHierPos; }
  void RoadQuery::setSurfaceScale(
      const double &wzKzI) { mSurfaceScale = wzKzI; }
  int RoadQuery::addTrackS(TrackCoord &
                               HQGFE,
                           const double &hg7Gm, bool SFH3M)
  {
    if (!SFH3M)
      return RESULT_ERROR;
    if (fabs(hg7Gm) < 1.0e-5)
      return RESULT_ON_ROAD;
    double rganP = hg7Gm;
    RoadHeader *wR6_y = trackId2Node(
        HQGFE);
    if (!wR6_y)
      return RESULT_ERROR;
    RailroadSwitch *VRE77 = reinterpret_cast<
        RailroadSwitch *>(wR6_y->getChild(ODR_OPCODE_RAILROAD_SWITCH));
    RailroadSwitch *
        IJhEC = 0;
    double O_xAY = 0.0;
    while (VRE77)
    {
      double zSXJc = VRE77->mS - HQGFE.getS();
      if (((
               fabs(zSXJc) < fabs(rganP)) &&
           (zSXJc * rganP) > 0.0) ||
          (fabs(zSXJc) < 1.0e-3))
      {
        if (!IJhEC || (fabs(zSXJc) < fabs(O_xAY)))
        {
          IJhEC = VRE77;
          O_xAY = zSXJc;
        }
      }
      VRE77 = reinterpret_cast<
          RailroadSwitch *>(VRE77->getRight());
    }
    if (IJhEC)
    {
      HQGFE.setS(HQGFE.getS() + O_xAY);
      rganP -= O_xAY;
      if (((rganP < 0.0) && (IJhEC->mDir == ODR_DIRECTION_MINUS)) || ((rganP > 0.0) && (IJhEC->mDir == ODR_DIRECTION_PLUS)))
      {
        if (IJhEC->mState ==
            ODR_RAILROAD_SWITCH_STATE_TURN)
        {
          HQGFE.setTrackId(IJhEC->mSideTrackId);
          HQGFE.setTrackId(IJhEC->mSideTrackIdAsString);
          HQGFE.setS(IJhEC->mSideTrackS);
          if (fabs(
                  IJhEC->mSideTrackS) > 0.5)
            rganP = -1.0 * fabs(rganP);
          else
            rganP = fabs(rganP);
        }
      }
      double
          QXmP6 = fabs(rganP);
      if (QXmP6 > 0.5)
        QXmP6 = 0.5;
      if (rganP < 0.0)
        QXmP6 *= -1.0;
      HQGFE.setS(
          HQGFE.getS() + QXmP6);
      bool dirChanged = false;
      if (track2validTrack(HQGFE, dirChanged,
                           SFH3M) != RESULT_ON_ROAD)
        return RESULT_ERROR;
      rganP -= QXmP6;
      if (dirChanged)
        rganP *= -1.0;
      return addTrackS(HQGFE, rganP, true);
    }
    double i2c70 = HQGFE.getS() + rganP;
    if ((
            i2c70 >= 0.0) &&
        (i2c70 <= wR6_y->mLength))
    {
      HQGFE.setS(i2c70);
      return RESULT_ON_ROAD;
    }
    if (i2c70 < 0.0)
    {
      if (i2c70 < -1.0)
      {
        HQGFE.setS(-1.0);
        rganP = i2c70 + 1.0;
      }
      else
      {
        HQGFE.setS(
            i2c70);
        rganP = 0.0;
      }
    }
    else if (i2c70 > wR6_y->mLength)
    {
      if (i2c70 > (wR6_y->mLength + 1.0))
      {
        HQGFE.setS(wR6_y->mLength + 1.0);
        rganP = i2c70 - wR6_y->mLength - 1.0;
      }
      else
      {
        HQGFE.setS(
            i2c70);
        rganP = 0.0;
      }
    }
    bool dirChanged = false;
    if (track2validTrack(HQGFE, dirChanged,
                         SFH3M) != RESULT_ON_ROAD)
      return RESULT_ERROR;
    if (dirChanged)
      rganP *= -1.0;
    return addTrackS(HQGFE, rganP, true);
  }
  void RoadQuery::setValidLaneTypes(bool ce0Qj, bool
                                                    lAdDx)
  {
    mAllowLaneTypeRail = lAdDx;
    mAllowLaneTypeCar = ce0Qj;
  }
  int RoadQuery::
      st2pitchAndZ(RoadHeader *wR6_y, const double &BJBDA, const double &rkiXc, double &s4E1B, double &hkK5C, const double &hgSHv)
  {
    hkK5C = 0.0;
    s4E1B = 0.0;
    if (!wR6_y)
      return RESULT_ERROR;
    Elevation *j6B6g = s2elevation(wR6_y, s2tolS(BJBDA, wR6_y));
    if (j6B6g)
    {
      hkK5C = j6B6g->s2z(s2tolS(BJBDA, wR6_y));
    }
    LateralShape *qDMed = EBWwH(wR6_y, s2tolS(BJBDA, wR6_y), rkiXc);
    if (qDMed)
    {
      hkK5C += cos(hgSHv) * (qDMed->st2z(s2tolS(BJBDA, wR6_y), rkiXc));
    }
    if (j6B6g)
      s4E1B = j6B6g->s2pitch(s2tolS(BJBDA, wR6_y));
    if (mSurfaceScale ==
        0.0)
      return RESULT_ON_ROAD;
    SurfaceCRG *oV99J = s2surface(wR6_y, s2tolS(BJBDA, wR6_y),
                                  SurfaceCRG::sPurposeElevation);
    if (oV99J)
    {
      if ((oV99J->mMode == oV99J->sModeAttached) || (oV99J->mMode == oV99J->sModeAttached0))
      {
        double Zbgoe = 0.0;
        double GbZcy = 0.0;
        oV99J
            ->st2zp(s2tolS(BJBDA, wR6_y), rkiXc, Zbgoe, GbZcy, true);
        if (mUseContactPatch)
        {
          double
              QS1VX = s2tolS(BJBDA, wR6_y);
          double nM0cp = 0.0;
          double HCZvL = 0.0;
          oV99J->st2zp(QS1VX +
                           0.5 * mContactPatchLength,
                       rkiXc - 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe +=
              nM0cp;
          GbZcy += HCZvL;
          oV99J->st2zp(QS1VX - 0.5 * mContactPatchLength, rkiXc - 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          oV99J->st2zp(
              QS1VX + 0.5 * mContactPatchLength, rkiXc + 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          oV99J->st2zp(QS1VX - 0.5 * mContactPatchLength, rkiXc + 0.5 * mContactPatchWidth, nM0cp, HCZvL, true);
          Zbgoe += nM0cp;
          GbZcy += HCZvL;
          Zbgoe /= 5.0;
          GbZcy /= 5.0;
        }
        Zbgoe *= mSurfaceScale;
        if (oV99J->mMode == oV99J->sModeAttached0)
          hkK5C = Zbgoe;
        else
          hkK5C += Zbgoe;
      }
      else
      {
        Coord wiBPl;
        s4E1B = 0.0;
        TrackCoord HQGFE(wR6_y->mId, BJBDA,
                         rkiXc);
        HQGFE.setTrackId(wR6_y->mIdAsString);
        if (track2inertialSimple(wiBPl, HQGFE,
                                 wR6_y) == RESULT_ON_ROAD)
        {
          oV99J->xy2zp(wiBPl.getX(), wiBPl.getY(), hkK5C, s4E1B, true);
          if (mUseContactPatch)
          {
            double dx = 0.5 * (mContactPatchLength * cos(wiBPl.getH()) -
                               mContactPatchWidth * sin(wiBPl.getH()));
            double dy = 0.5 * (mContactPatchLength * sin(
                                                         wiBPl.getH()) +
                               mContactPatchWidth * cos(wiBPl.getH()));
            double nM0cp = 0.0;
            double
                HCZvL = 0.0;
            oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() + dy, nM0cp, HCZvL, true);
            hkK5C +=
                nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() + dy, nM0cp, HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() + dx, wiBPl.getY() - dy, nM0cp,
                         HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            oV99J->xy2zp(wiBPl.getX() - dx, wiBPl.getY() - dy, nM0cp, HCZvL, true);
            hkK5C += nM0cp;
            s4E1B += HCZvL;
            hkK5C /= 5.0;
            s4E1B /= 5.0;
          }
          hkK5C *=
              mSurfaceScale;
        }
      }
    }
    return RESULT_ON_ROAD;
  }
  GeoHeader *RoadQuery::track2geoHeader(
      const TrackCoord &HQGFE)
  {
    RoadHeader *wR6_y = trackId2Node(HQGFE);
    return track2geoHeader(wR6_y, HQGFE);
  }
  GeoHeader *RoadQuery::track2geoHeader(RoadHeader *
                                            wR6_y,
                                        const TrackCoord &HQGFE)
  {
    if (!wR6_y)
      return 0;
    GeoHeader *xjOaT =
        reinterpret_cast<GeoHeader *>(mHierPos.findNode(ODR_OPCODE_GEO_HEADER));
    double
        DFg6k = s2tolS(HQGFE.getS(), wR6_y);
    if (DFg6k == wR6_y->mLength)
      DFg6k -= mSTolerance;
    if (
        xjOaT && xjOaT->mS <= DFg6k && xjOaT->mSEnd >= DFg6k)
      return xjOaT;
    if (!xjOaT)
      xjOaT = wR6_y
                  ->getFirstGeoHeader();
    if (xjOaT)
    {
      if (xjOaT->mS > DFg6k)
      {
        while (xjOaT && xjOaT->mS > DFg6k)
          xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getLeft());
      }
      else
      {
        while (xjOaT && xjOaT->mSEnd < DFg6k)
          xjOaT = reinterpret_cast<GeoHeader *>(xjOaT->getRight());
      }
    }
    mHierPos.registerNode(xjOaT, ODR_OPCODE_GEO_HEADER);
    return xjOaT;
  }
  Elevation *RoadQuery::
      s2elevation(RoadHeader *wR6_y, const double &BJBDA)
  {
    if (!wR6_y)
      return 0;
    Elevation *
        j6B6g = reinterpret_cast<Elevation *>(mHierPos.findNode(ODR_OPCODE_ELEVATION));
    if (
        j6B6g && j6B6g->mS <= BJBDA && j6B6g->mSEnd >= BJBDA)
      return j6B6g;
    if (!j6B6g)
      j6B6g = wR6_y
                  ->getFirstElevation();
    if (j6B6g)
    {
      if (j6B6g->mS > BJBDA)
      {
        while (j6B6g && j6B6g->mS > BJBDA)
          j6B6g = reinterpret_cast<Elevation *>(j6B6g->getLeft());
      }
      else
      {
        while (j6B6g && j6B6g->mSEnd < BJBDA)
          j6B6g = reinterpret_cast<Elevation *>(j6B6g->getRight());
      }
    }
    mHierPos.registerNode(j6B6g, ODR_OPCODE_ELEVATION);
    return j6B6g;
  }
  LateralShape *RoadQuery::
      EBWwH(RoadHeader *wR6_y, const double &BJBDA, const double &rkiXc)
  {
    if (!wR6_y)
      return 0;
    LateralShape *qDMed = reinterpret_cast<LateralShape *>(mHierPos.findNode(
        ODR_OPCODE_LATERAL_SHAPE));
    if (qDMed && (qDMed->mS <= BJBDA) && (qDMed->mSEnd >= BJBDA) &&
        (qDMed->mT <= rkiXc) && (qDMed->mTEnd >= rkiXc))
      return qDMed;
    if (!qDMed)
      qDMed = wR6_y->getFirstLateralShape();
    if (qDMed)
    {
      if (qDMed->mS > BJBDA)
      {
        while (qDMed && (qDMed->mS >
                         BJBDA))
          qDMed = reinterpret_cast<LateralShape *>(qDMed->getLeft());
      }
      else
      {
        while (qDMed && (qDMed->mSEnd < BJBDA))
          qDMed = reinterpret_cast<LateralShape *>(qDMed->getRight());
      }
    }
    if (qDMed)
    {
      LateralShape *kxyKV = reinterpret_cast<LateralShape *>(qDMed->getLeft());
      while (kxyKV)
      {
        if (kxyKV->mS != qDMed->mS)
          break;
        qDMed = kxyKV;
        kxyKV = reinterpret_cast<
            LateralShape *>(qDMed->getLeft());
      }
    }
    while (qDMed)
    {
      if (qDMed->mT > rkiXc)
      {
        qDMed = 0;
        break;
      }
      if ((qDMed->mTEnd == qDMed->mT) || ((rkiXc <= qDMed->mTEnd) && (rkiXc >= qDMed->mT)))
        break;
      LateralShape *kxyKV = reinterpret_cast<LateralShape *>(qDMed->getRight());
      if (
          kxyKV)
      {
        if (kxyKV->mS != qDMed->mS)
          break;
      }
      qDMed = kxyKV;
    }
    mHierPos.registerNode(qDMed,
                          ODR_OPCODE_LATERAL_SHAPE);
    return qDMed;
  }
  SurfaceCRG *RoadQuery::s2surface(
      RoadHeader *wR6_y, const double &BJBDA, const unsigned short &TCz3u)
  {
    if (!wR6_y)
      return 0;
    if (!(wR6_y->hasSurfaceData(TCz3u)))
      return 0;
    SurfaceCRG *oV99J = reinterpret_cast<SurfaceCRG *>(mHierPos.findNode(ODR_OPCODE_SURFACE_CRG));
    if (oV99J)
    {
      if ((oV99J->mPurpose & TCz3u) && (oV99J->mS <= BJBDA) && (oV99J->mSEnd >= BJBDA))
        return oV99J;
    }
    oV99J =
        wR6_y->s2surface(BJBDA, TCz3u);
    mHierPos.registerNode(oV99J, ODR_OPCODE_SURFACE_CRG);
    return oV99J;
  }
  Superelevation *RoadQuery::s2superelevation(RoadHeader *wR6_y,
                                              const double &BJBDA)
  {
    if (!wR6_y)
      return 0;
    Superelevation *XUz25 = reinterpret_cast<
        Superelevation *>(mHierPos.findNode(ODR_OPCODE_SUPERELEVATION));
    if (XUz25 && XUz25->mS <= BJBDA && XUz25->mSEnd >= BJBDA)
      return XUz25;
    if (!XUz25)
      XUz25 = wR6_y->getFirstSuperelevation();
    if (XUz25)
    {
      if (XUz25->mS > BJBDA)
      {
        while (XUz25 && XUz25->mS >
                            BJBDA)
          XUz25 = reinterpret_cast<Superelevation *>(XUz25->getLeft());
      }
      else
      {
        while (
            XUz25 && XUz25->mSEnd < BJBDA)
          XUz25 = reinterpret_cast<Superelevation *>(XUz25->getRight());
      }
    }
    mHierPos.registerNode(XUz25, ODR_OPCODE_SUPERELEVATION);
    return XUz25;
  }
  int RoadQuery::track2InertialPitchAndZ(RoadHeader *wR6_y, Coord &Tlj7t, const TrackCoord &HQGFE)
  {
    double s4E1B;
    double hkK5C;
    int result;
    if ((result =
             trackAndRoll2PitchAndZ(wR6_y, HQGFE, Tlj7t.getR(), s4E1B, hkK5C)) != RESULT_ON_ROAD)
      return result;
    Tlj7t.setP(s4E1B + HQGFE.getP());
    Tlj7t.setZ(hkK5C + HQGFE.getZ());
    return RESULT_ON_ROAD;
  }
  int RoadQuery::trackAndRoll2PitchAndZ(RoadHeader *wR6_y,
                                        const TrackCoord &HQGFE, const double &hgSHv, double &s4E1B, double &hkK5C)
  {
    int result;
    if (!Ekmkk(wR6_y, HQGFE, hkK5C))
    {
      double ux1Mc = s2tolS(HQGFE.getS(), wR6_y);
      if (ux1Mc ==
          wR6_y->mLength)
        ux1Mc -= mSTolerance;
      if ((result = st2pitchAndZ(wR6_y, ux1Mc, HQGFE.getT(), s4E1B, hkK5C, hgSHv)) != RESULT_ON_ROAD)
        return result;
      hkK5C += HQGFE.getT() * sin(
                                  hgSHv);
    }
    else
      s4E1B = 0.0;
    return RESULT_ON_ROAD;
  }
  double RoadQuery::getLaneWidth(
      Lane *lane, const double &rganP)
  {
    double EqgCd;
    double SIMcL;
    return (getLaneWidth(lane, rganP, EqgCd, SIMcL));
  }
  double RoadQuery::getLaneWidth(Lane *lane, const double &rganP, double &lseyi, double &L9KFn)
  {
    lseyi = 0.0;
    L9KFn = 0.0;
    if (!lane)
      return 0.0;
    if (lane
            ->mId == 0)
      return 0.0;
    LaneWidth *qfK7D = lane->getFirstWidth();
    while (qfK7D && (qfK7D->mOffsetEnd < rganP))
      qfK7D = reinterpret_cast<LaneWidth *>(qfK7D->getRight());
    if (!qfK7D)
    {
      LaneBorder *p6sIa = lane->getFirstBorder();
      while (p6sIa && (p6sIa->mOffsetEnd <
                       rganP))
        p6sIa = reinterpret_cast<LaneBorder *>(p6sIa->getRight());
      if (!p6sIa)
        return 0.0;
      lseyi = p6sIa->ds2widthDot(rganP);
      L9KFn = p6sIa->ds2Curv(rganP);
      return p6sIa->ds2width(rganP);
    }
    lseyi = qfK7D->ds2widthDot(rganP);
    L9KFn = qfK7D->ds2Curv(rganP);
    return qfK7D->ds2width(rganP);
  }
  double RoadQuery::getLaneWidthAndRoadMark(Lane *
                                                lane,
                                            const double &rganP)
  {
    mRoadMark = getRoadMark(lane, rganP);
    return getLaneWidth(
        lane, rganP);
  }
  unsigned short RoadQuery::getRoadMark(Lane *lane, const double &rganP)
  {
    mHierPos.registerNode(0, ODR_OPCODE_LANE_ROAD_MARK);
    if (!lane)
      return ODR_ROAD_MARK_TYPE_NONE;
    RoadMark *Ai0ce;
    RoadMark *u024a = lane->getFirstRoadMark();
    if (!u024a)
      return ODR_ROAD_MARK_TYPE_NONE;
    else if (u024a->mOffset > rganP)
      return ODR_ROAD_MARK_TYPE_NONE;
    do
    {
      Ai0ce = u024a;
      u024a = reinterpret_cast<RoadMark *>(Ai0ce->getRight());
    } while (u024a && u024a->mOffset < rganP);
    mHierPos.registerNode(Ai0ce,
                          ODR_OPCODE_LANE_ROAD_MARK);
    return Ai0ce->mType;
  }
  bool RoadQuery::checkForRoadData()
  {
    if (mExplicitRoadData)
      return mRoadData != 0;
    if (mRoadData && (mRoadData == RoadData::
                                       getInstance()))
      return true;
    mRoadData = RoadData::getInstance();
    if (!mRoadData)
    {
      std ::cerr << "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x63\x68\x65\x63\x6b\x46\x6f\x72\x52\x6f\x61\x64\x44\x61\x74\x61\x3a\x20\x57\x41\x52\x4e\x49\x4e\x47\x3a\x20\x52\x6f\x61\x64\x20\x44\x61\x74\x61\x20\x69\x73\x20\x6e\x6f\x74\x20\x28\x79\x65\x74\x29\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64\x21\x20\x51\x75\x65\x72\x69\x65\x73\x20\x61\x72\x65\x20\x6e\x6f\x74\x20\x79\x65\x74\x20\x66\x65\x61\x73\x69\x62\x6c\x65\x2e"
                 << std::endl;
      return false;
    }
    return true;
  }
  int RoadQuery::getLaneOnPreviousRoad(
      const LaneCoord &Hv9fs, RoadHeader *wR6_y)
  {
    bool OjzDY = false;
    return getLaneOnPreviousRoad(Hv9fs, wR6_y, OjzDY);
  }
  int RoadQuery::getLaneOnPreviousRoad(
      const LaneCoord &Hv9fs, RoadHeader *wR6_y, bool &OjzDY)
  {
    OjzDY = false;
    if (!wR6_y)
      wR6_y =
          trackId2Node(Hv9fs);
    if (!wR6_y)
      return 0;
    LaneSection *H4prs = wR6_y->getFirstLaneSection();
    if (!H4prs)
      return 0;
    Lane *lane = getLaneFromId(H4prs, Hv9fs.getLaneId());
    if (!lane)
      return 0;
    LaneLink *SJBOF = lane->getPredecessorLink();
    if (!SJBOF)
    {
      RoadHeader *II6ml = wR6_y->getPredecessor(true);
      if (II6ml)
      {
        JuncHeader *GqNs7 =
            reinterpret_cast<JuncHeader *>(II6ml->getJunction());
        if (GqNs7)
        {
          JuncLink *vuhmI =
              GqNs7->getFirstLink();
          while (vuhmI)
          {
            JuncLaneLink *DPG7X = vuhmI->getFirstLaneLink();
            while (DPG7X)
            {
              if ((DPG7X->mIncomingLane->mId == Hv9fs.getLaneId()) || ((Hv9fs.getLaneId() == 0) && (DPG7X->mConnectingLane->mId == -1)))
              {
                OjzDY = true;
                if (Hv9fs.getLaneId() == 0)
                  return 0;
                return DPG7X->mConnectingLane->mId;
              }
              DPG7X =
                  reinterpret_cast<JuncLaneLink *>(DPG7X->getRight());
            }
            vuhmI = reinterpret_cast<
                JuncLink *>(vuhmI->getRight());
          }
        }
        else
        {
          if (Hv9fs.getLaneId() == 0)
          {
            OjzDY = true;
            return 0;
          }
        }
      }
      return 0;
    }
    OjzDY = true;
    return SJBOF->mLaneId;
  }
  int RoadQuery::
      getLaneOnNextRoad(const LaneCoord &Hv9fs, RoadHeader *wR6_y)
  {
    bool OjzDY = false;
    return getLaneOnNextRoad(Hv9fs, wR6_y, OjzDY);
  }
  int RoadQuery::getLaneOnNextRoad(
      const LaneCoord &Hv9fs, RoadHeader *wR6_y, bool &OjzDY)
  {
    OjzDY = false;
    if (!wR6_y)
      wR6_y =
          trackId2Node(Hv9fs);
    if (!wR6_y)
      return 0;
    LaneSection *H4prs = wR6_y->getLastLaneSection();
    if (!H4prs)
      return 0;
    Lane *lane = getLaneFromId(H4prs, Hv9fs.getLaneId());
    if (!lane)
      return 0;
    LaneLink *SJBOF = lane->getSuccessorLink();
    if (!SJBOF)
    {
      RoadHeader *NGRHf = wR6_y->getSuccessor(true);
      if (NGRHf)
      {
        JuncHeader *GqNs7 =
            reinterpret_cast<JuncHeader *>(NGRHf->getJunction());
        if (GqNs7)
        {
          JuncLink *vuhmI =
              GqNs7->getFirstLink();
          while (vuhmI)
          {
            JuncLaneLink *DPG7X = vuhmI->getFirstLaneLink();
            while (DPG7X)
            {
              if (DPG7X->mIncomingLane->mId == Hv9fs.getLaneId())
              {
                OjzDY = true;
                return DPG7X->mConnectingLane->mId;
              }
              DPG7X = reinterpret_cast<JuncLaneLink *>(DPG7X->getRight());
            }
            vuhmI = reinterpret_cast<JuncLink *>(vuhmI->getRight());
          }
        }
        else
        {
          if (
              Hv9fs.getLaneId() == 0)
          {
            OjzDY = true;
            return Hv9fs.getLaneId();
          }
        }
      }
      return 0;
    }
    OjzDY =
        true;
    return SJBOF->mLaneId;
  }
  Lane *RoadQuery::getLaneFromId(LaneSection *H4prs, int
                                                         id)
  {
    if (!H4prs)
      return 0;
    return H4prs->getLaneFromId(id);
  }
  int RoadQuery::
      inertial2lane(RoadHeader *wR6_y, GeoHeader *xjOaT, const Coord &wiBPl, LaneCoord &Hv9fs, bool oWK6t)
  {
    static const bool PwJ1X = false;
    LaneCoord wxjcS;
    if (!wR6_y || !xjOaT)
      return RESULT_NOT_ON_ROAD;
    if (PwJ1X)
      fprintf(stderr,
              "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x63\x68\x65\x63\x6b\x69\x6e\x67\x20\x72\x6f\x61\x64\x20\x25\x64"
              "\n",
              wR6_y->mId);
    GeoNode *pWj2K = reinterpret_cast<GeoNode *>(xjOaT->getChild());
    if (!pWj2K)
      return RESULT_ERROR;
    if (PwJ1X)
      fprintf(stderr,
              "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x67\x65\x6f\x48\x64\x72\x2d\x3e\x69\x6e\x42\x6f\x75\x6e\x64\x69\x6e\x67\x42\x6f\x78\x20\x3d\x20\x25\x64\x2c\x20\x67\x4e\x6f\x64\x65\x2d\x3e\x63\x6f\x6e\x74\x61\x69\x6e\x73\x50\x6f\x73\x20\x3d\x20\x25\x64"
              "\n",
              xjOaT->inBoundingBox(wiBPl.getX(), wiBPl.getY()), pWj2K->containsPos(wiBPl.getX(), wiBPl.getY()));
    if (xjOaT->inBoundingBox(wiBPl.getX(), wiBPl.getY()) && pWj2K->containsPos(wiBPl.getX(), wiBPl.getY()))
    {
      double BJBDA;
      double rkiXc;
      double wYv5i = (1.2 * wR6_y->mMaxHalfWidth) > mMaxTrackDist ? (1.2 * wR6_y->mMaxHalfWidth) : mMaxTrackDist;
      bool _CPxI = pWj2K->xy2st(wiBPl.getX(), wiBPl.getY(), BJBDA, rkiXc);
      if (_CPxI && fabs(
                       rkiXc) <= wYv5i)
      {
        wxjcS.setTrackId(wR6_y->mId);
        wxjcS.setTrackId(wR6_y->mIdAsString);
        wxjcS.setS(BJBDA);
        double aOzQT;
        pWj2K->s2h(BJBDA, aOzQT);
        double hgSHv = 0.0;
        if (
            getRoll(wR6_y, wxjcS, hgSHv) == RESULT_ON_ROAD)
          rkiXc /= cos(hgSHv);
        wxjcS.setT(rkiXc);
        if (PwJ1X)
        {
          fprintf(stderr,
                  "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x66\x6f\x75\x6e\x64\x20\x70\x6f\x73\x20\x6f\x6e\x20\x72\x6f\x61\x64\x20\x25\x64\x2c\x20\x73\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x74\x20\x3d\x20\x25\x2e\x33\x6c\x66\x2c\x20\x72\x6f\x6c\x6c\x20\x3d\x20\x25\x2e\x33\x66\x3a\x20"
                  "\n",
                  wR6_y->mId, BJBDA, rkiXc, hgSHv);
          pWj2K->print();
        }
        if (track2lane(wxjcS, wR6_y) ==
            RESULT_ON_ROAD)
        {
          if (PwJ1X)
            fprintf(stderr,
                    "\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x66\x6f\x75\x6e\x64\x20\x6c\x61\x6e\x65\x20"
                    "\n");
          mCurvature = geoNode2curvature(pWj2K, wR6_y, wxjcS.getS());
          double s4E1B;
          double
              hkK5C;
          int result;
          if ((result = trackAndRoll2PitchAndZ(wR6_y, wxjcS, hgSHv, s4E1B, hkK5C)) != RESULT_ON_ROAD)
            return result;
          if (oWK6t)
          {
            Node *qWeBE = mHierPos.findNode(
                ODR_OPCODE_LANE);
            mHierPos.keepNode(wR6_y);
            mHierPos.keepNode(xjOaT);
            mHierPos.keepNode(qWeBE);
          }
          Hv9fs = wxjcS;
          if (mUseLaneHeight)
            hkK5C += lane2laneHeight(Hv9fs);
          mFootPoint.set(wiBPl.getX(), wiBPl.getY(), hkK5C, aOzQT, s4E1B, hgSHv);
          Hv9fs.setZ(
              wiBPl.getZ() - hkK5C);
          Hv9fs.setP(wiBPl.getP() - s4E1B);
          Hv9fs.setR(wiBPl.getR() - hgSHv);
          Hv9fs.setH(wiBPl.getH() - aOzQT + mLaneDeltaHdg);
          return result;
        }
        else if (PwJ1X)
          fprintf(stderr,
                  "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x52\x6f\x61\x64\x51\x75\x65\x72\x79\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x6c\x61\x6e\x65\x3a\x20\x74\x72\x61\x63\x6b\x32\x6c\x61\x6e\x65\x20\x66\x61\x69\x6c\x65\x64\x20"
                  "\n");
      }
    }
    return RESULT_NOT_ON_ROAD;
  }
  Lane *RoadQuery::laneId2Node(int laneId)
  {
    Lane *lane = reinterpret_cast<Lane *>(mHierPos.findNode(ODR_OPCODE_LANE));
    if (lane && lane->mId == laneId)
      return lane;
    return 0;
  }
  const double &RoadQuery::s2tolS(const double &BJBDA, RoadHeader *REKut)
  {
    static double QulVA = 0.0;
    if (!REKut)
      return BJBDA;
    if ((BJBDA < 0.0) && (BJBDA > -mSTolerance))
      return QulVA;
    if ((BJBDA > REKut->mLength) && (BJBDA < (REKut->mLength + mSTolerance)))
      return REKut->mLength;
    return BJBDA;
  }
  bool RoadQuery::laneSpeedFromRoadType(const LaneCoord &Hv9fs, double &kMjGG)
  {
    RoadHeader *REKut =
        reinterpret_cast<RoadHeader *>(mHierPos.findNode(ODR_OPCODE_ROAD_HEADER));
    if (!REKut)
      return false;
    int v7m2w = getRoadType(Hv9fs);
    RoadType *ibiOz = reinterpret_cast<
        RoadType *>(mHierPos.findNode(ODR_OPCODE_ROAD_TYPE));
    if (ibiOz)
    {
      RoadSpeed *TG8hT =
          reinterpret_cast<RoadSpeed *>(ibiOz->getChild(ODR_OPCODE_ROAD_SPEED));
      if (TG8hT)
      {
        kMjGG = TG8hT->mMax;
        return true;
      }
    }
    switch (v7m2w)
    {
    case ODR_ROAD_TYPE_NONE:
      return false;
    case ODR_ROAD_TYPE_RURAL:
      kMjGG = 100.0 / 3.6;
      return true;
    case ODR_ROAD_TYPE_MOTORWAY:
      kMjGG = 250.0 / 3.6;
      return true;
    case ODR_ROAD_TYPE_TOWN:
    case ODR_ROAD_TYPE_TOWN_EXPRESSWAY:
    case ODR_ROAD_TYPE_TOWN_COLLECTOR:
    case ODR_ROAD_TYPE_TOWN_ARTERIAL:
    case ODR_ROAD_TYPE_TOWN_PRIVATE:
    case ODR_ROAD_TYPE_TOWN_LOCAL:
      kMjGG = 50.0 / 3.6;
      return true;
    case ODR_ROAD_TYPE_LOW_SPEED:
      kMjGG = 30.0 / 3.6;
      return true;
    case ODR_ROAD_TYPE_PEDESTRIAN:
      kMjGG = 6.0 / 3.6;
      return true;
    default:
      return false;
    }
    return false;
  }
  double RoadQuery::lane2laneHeight(const LaneCoord &Hv9fs)
  {
    mLaneHeight = 0.0;
    Lane *lane = reinterpret_cast<Lane *>(mHierPos.findNode(ODR_OPCODE_LANE));
    if (!lane)
      return 0.0;
    LaneHeight *lHeight =
        reinterpret_cast<LaneHeight *>(lane->getChild(ODR_OPCODE_LANE_HEIGHT));
    if (!lHeight)
      return 0.0;
    while (lHeight)
    {
      if (lHeight->mS <= Hv9fs.getS() && lHeight->mSEnd >=
                                             Hv9fs.getS())
      {
        double ZwrVB = 0.0;
        if (fabs(mLaneWidth) > 1.0e-5)
          ZwrVB = Hv9fs.getOffset() / mLaneWidth + 0.5;
        if (Hv9fs.getLaneId() < 0)
          ZwrVB = 1.0 - ZwrVB;
        mLaneHeight = lHeight->sAndRelOffset2height(Hv9fs.getS(), ZwrVB);
        return mLaneHeight;
      }
      lHeight =
          reinterpret_cast<LaneHeight *>(lHeight->getRight());
    }
    return 0.0;
  }
  bool RoadQuery::
      inTunnel(const LaneCoord &Hv9fs)
  {
    RoadHeader *REKut = trackId2Node(Hv9fs);
    if (!REKut)
      return false;
    Tunnel *jVZEe = REKut->getFirstTunnel();
    while (jVZEe)
    {
      if (jVZEe->mS <=
          Hv9fs.getS())
      {
        if ((jVZEe->mS + jVZEe->mLength) >= Hv9fs.getS())
          return true;
      }
      jVZEe =
          reinterpret_cast<Tunnel *>(jVZEe->getRight());
    }
    return false;
  }
  bool RoadQuery::
      onBridge(const LaneCoord &Hv9fs)
  {
    RoadHeader *REKut = trackId2Node(Hv9fs);
    if (!REKut)
      return false;
    Bridge *ScbdT = REKut->getFirstBridge();
    while (ScbdT)
    {
      if (ScbdT->mS <=
          Hv9fs.getS())
      {
        if ((ScbdT->mS + ScbdT->mLength) >= Hv9fs.getS())
          return true;
      }
      ScbdT =
          reinterpret_cast<Bridge *>(ScbdT->getRight());
    }
    return false;
  }
  int RoadQuery::
      addLaneS(LaneCoord &Hv9fs, const double &rganP)
  {
    static const double PjSg_ = 1.0e-4;
    LaneSection *H4prs = track2laneSection(Hv9fs);
    if (!H4prs)
      return RESULT_NOT_ON_ROAD;
    int fuJOD = 0;
    if (rganP >= 0.0)
    {
      if (H4prs->mSEnd >= (Hv9fs.getS() + rganP - PjSg_))
      {
        Hv9fs.setS(Hv9fs.getS() + rganP);
        return RESULT_ON_ROAD;
      }
      if (!(H4prs->getRight()))
        fuJOD = 1;
    }
    else
    {
      if (H4prs->mS <= (Hv9fs.getS() + rganP + PjSg_))
      {
        Hv9fs.setS(Hv9fs.getS() + rganP);
        return RESULT_ON_ROAD;
      }
      if (!(H4prs->getLeft()))
        fuJOD = 2;
    }
    if (!fuJOD)
    {
      Lane *lane =
          reinterpret_cast<Lane *>(H4prs->getChild());
      while (lane)
      {
        if (lane->mId == Hv9fs.getLaneId())
          break;
        lane = reinterpret_cast<Lane *>(lane->getRight());
      }
      if (!lane)
        return RESULT_NOT_ON_ROAD;
      if (rganP >= 0.0)
      {
        if (lane->getSuccessor())
        {
          double RFDHF =
              rganP - (H4prs->mSEnd - Hv9fs.getS());
          Hv9fs.setS(H4prs->mSEnd);
          Hv9fs.setLaneId(lane
                              ->getSuccessor()
                              ->mId);
          if (RFDHF > 1.e-3)
          {
            Hv9fs.setS(H4prs->mSEnd + 1.e-3);
            RFDHF -=
                1.e-3;
            return addLaneS(Hv9fs, RFDHF);
          }
          return RESULT_ON_ROAD;
        }
        else
          return RESULT_NOT_ON_ROAD;
      }
      else
      {
        if (lane->getPredecessor())
        {
          double RFDHF = rganP + (Hv9fs.getS() - H4prs->mS);
          Hv9fs.setS(H4prs->mS);
          Hv9fs.setLaneId(lane->getPredecessor()->mId);
          if (RFDHF < -1.e-3)
          {
            Hv9fs.setS(H4prs->mS - 1.e-3);
            RFDHF += 1.e-3;
            return addLaneS(
                Hv9fs, RFDHF);
          }
          return RESULT_ON_ROAD;
        }
        else
          return RESULT_NOT_ON_ROAD;
      }
    }
    if (fuJOD)
    {
      Hv9fs.setS(Hv9fs.getS() + rganP);
      int retVal = lane2validLane(Hv9fs);
      return (retVal ==
              RESULT_ON_ROAD)
                 ? retVal
                 : RESULT_NOT_ON_ROAD;
    }
    return RESULT_NOT_ON_ROAD;
  }
  void
  RoadQuery::setContactPatchDimension(const double &length, const double &width)
  {
    mContactPatchLength = fabs(length);
    mContactPatchWidth = fabs(width);
    mUseContactPatch = (mContactPatchLength > 1.0 - 3) || (mContactPatchWidth > 1.0e-3);
  }
  void RoadQuery::
      useLaneHeight(bool F744N) { mUseLaneHeight = F744N; }
  void RoadQuery::setZOptimization(bool F744N, const float &AiTnB)
  {
    mMinimizeDeltaZ = F744N;
    mZTolerance = AiTnB;
  }
  bool
  RoadQuery::isOptimumSolution() const { return mOptimumSolution; }
  bool RoadQuery::
      Ekmkk(RoadHeader *REKut, const TrackCoord &HQGFE, double &hkK5C)
  {
    if (!REKut)
      return false;
    JuncHeader *rQQS4 = reinterpret_cast<JuncHeader *>(REKut->getJunction());
    if (!rQQS4)
      return 0;
    SurfaceCRG *SgQ7l = rQQS4->getSurface(SurfaceCRG::sPurposeElevation);
    if (!SgQ7l)
      return 0;
    double s4E1B = 0.0;
    Coord wiBPl;
    if (track2inertialSimple(wiBPl,
                             HQGFE, REKut) == RESULT_ON_ROAD)
      return SgQ7l->xy2zp(wiBPl.getX(), wiBPl.getY(), hkK5C, s4E1B, false);
    return false;
  }
  double RoadQuery::geoNode2curvature(GeoNode *pWj2K,
                                      RoadHeader *REKut, const double &BJBDA)
  {
    double retVal = 0.0;
    if (!pWj2K || !REKut)
      return 0.0;
    retVal = pWj2K->s2curvature(s2tolS(BJBDA, REKut));
    static bool fKr1X = false;
    if (
        fKr1X)
    {
      LaneOffset *hYLzo = reinterpret_cast<LaneOffset *>(REKut->getChild(
          ODR_OPCODE_LANE_OFFSET));
      while (hYLzo && (hYLzo->mSEnd < BJBDA))
        hYLzo =
            reinterpret_cast<LaneOffset *>(hYLzo->getRight());
      if (hYLzo)
        retVal += hYLzo->s2curvature(BJBDA);
    }
    return retVal;
  }
} // namespace OpenDrive
