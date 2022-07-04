#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "posInfo.hpp"

#include <algorithm>
#include <iostream>
#include <cmath>

using namespace std;
using namespace OpenDrive;

getPosInfo::getPosInfo() {}

// jm
bool getPosInfo::init(const std::string& odrFile)
{
    bool res = m_manager.loadFile(odrFile);
    return res;
}

// TODO: if the point is not in any roadjm/gx
bool getPosInfo::getInertialPosInfo(Point p)
{

    OpenDrive::Position* pos = m_manager.createPosition();
    m_manager.activatePosition(pos);
    m_manager.setInertialPos(p.x, p.y, p.z);
    bool result = m_manager.inertial2lane();
    if (result) {
        OpenDrive::RoadHeader* header = m_manager.getRoadHeader();
        if (!header)
            return false;
        m_posInfo.roadId = header->getId();
        m_posInfo.road = header;
        m_posInfo.roadlength = header->mLength;

        OpenDrive::Lane* lane = m_manager.getLane();
        if (!lane)
            return false;
        m_posInfo.lane = lane;
        double speed = m_manager.getLaneSpeed();
        m_posInfo.laneId = lane->getId();
        m_posInfo.laneSpeed = speed;
        m_posInfo.laneType = lane->getType();
        m_posInfo.laneWidth = m_manager.getLaneWidth();

        OpenDrive::LaneSection* section = m_manager.getLaneSection();
        m_posInfo.section = section;
        if (!section)
            return false;
        m_posInfo.sectionS = section->mS;

        m_posInfo.junctionId = m_manager.getJunctionId();

        OpenDrive::TrackCoord posTrack = m_manager.getTrackPos();
        m_posInfo.trackS = posTrack.getS();
        m_posInfo.trackT = posTrack.getT();
        m_posInfo.roadheading = posTrack.getH();

        OpenDrive::Elevation* posElevation = m_manager.getElevation();
        m_posInfo.ElvmS = posElevation->mS;
        m_posInfo.ElvmSEnd = posElevation->mSEnd;
        m_posInfo.ElvmA = posElevation->mA;
        m_posInfo.ElvmB = posElevation->mB;
        m_posInfo.ElvmC = posElevation->mC;
        m_posInfo.ElvmD = posElevation->mD;
    } else {
        std::cout << "the point is not on road." << std::endl;
    }
    return result;
}

// is Turning:gx
bool getPosInfo::isInSce2_46_7(Point p)
{
    bool   isInRoad = getInertialPosInfo(p);
    double pi = atan(1) * 4;
    double headingChangeMin = (pi / 36);
    {
        double                slength = m_posInfo.roadlength;
        OpenDrive::OdrManager Manager;
        OpenDrive::Position*  pos1 = Manager.createPosition();
        Manager.activatePosition(pos1);
        Manager.setLanePos(m_posInfo.roadId, m_posInfo.laneId, 0);
        Manager.lane2inertial();
        double              p1x = Manager.getInertialPos().getX();
        double              p1y = Manager.getInertialPos().getY();
        double              p1z = Manager.getInertialPos().getZ();
        OpenDrive::GeoCoord myCoord(p1x, p1y, p1z);
        Manager.setPos(myCoord);
        double  startH = myCoord.getH();
        posInfo pos_info1 = m_posInfo;
        while (pos_info1.section->getRight()) {
            OpenDrive::LaneSection* secInfo1 = pos_info1.section;
            OpenDrive::LaneSection* nextSecInfo1 =
                (OpenDrive::LaneSection*)secInfo1->getRight();
            pos_info1.section = nextSecInfo1;
            pos_info1.sectionS = pos_info1.section->mS;
            if (pos_info1.lane->getSuccessorLink()) {
                pos_info1.lane = pos_info1.lane->getSuccessor();
                pos_info1.laneId = pos_info1.lane->mId;
            }
            //
            else {
                break;
            }
        }
        OpenDrive::OdrManager myManager;
        OpenDrive::Position*  pos2 = myManager.createPosition();
        myManager.activatePosition(pos2);
        myManager.setLanePos(m_posInfo.roadId, pos_info1.lane->mId, slength);
        myManager.lane2inertial();
        double              p2x = myManager.getInertialPos().getX();
        double              p2y = myManager.getInertialPos().getY();
        double              p2z = myManager.getInertialPos().getZ();
        OpenDrive::GeoCoord myCoord2(p2x, p2y, p2z);
        myManager.setPos(myCoord2);
        const OpenDrive::GeoCoord& myCoord22 = myCoord2;
        double                     endH = myCoord22.getH();
        if ((endH == 0) && (startH == 0)) {
            double headingChange = atan((p2x - p1x) / (p2y - p1y));
            if ((abs(headingChange)) > headingChangeMin) {
                return 1;
            }
        } else if (startH != endH) {
            return 1;
        }
    }
    return 0;
}
