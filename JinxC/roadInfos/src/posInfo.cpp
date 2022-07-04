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
        // OpenDrive::OdrManager myManager;
        // OpenDrive::GeoCoord myCoord(p.x, p.y, p.z);
        // myManager.setPos( myCoord );
        // m_posInfo.roadheading = myCoord.getH();
        // OpenDrive::OdrManager myManager;
        // OpenDrive::Position* pos = myManager.createPosition();
        // myManager.activatePosition(pos);

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
        // std::cout << "the point is not on road." << std::endl;
    }
    return result;
}

// gx
int** getPosInfo::GetJuncDir(Point p1, float heading1)
{
    int** a;
    a = (int**)malloc(10 * sizeof(int*));
    for (int ai = 0; ai < 10; ai++) {
        a[ai] = (int*)malloc(2 * sizeof(int));
    }
    int ai = 0;

    int bi = 0;

    float pi = 4 * atan(1);
    bool  isInRoad = getInertialPosInfo(p1);
    if (!isInRoad) {
        a[0][0] = -1;
        return a;
    }
    posInfo pos_info1 = m_posInfo;
    pos_info1.roadId = pos_info1.road->mId;
    pos_info1.laneId = pos_info1.lane->mId;
    // // roadheading = myCoord.getH();
    // std::cout << "Preprocess enable :pos_info1.roadId is "
    // <<pos_info1.roadId<<std::endl; std::cout << "Preprocess enable
    // :pos_info1.laneId is " <<pos_info1.laneId<<std::endl;
    // // //if junction
    // // if (((pos_info1.roadlength-pos_info1.trackS<20)
    // // &&(pos_info1.road->getSuccessor())
    // // &&(pos_info1.road->getSuccessorLink()->getElemType() ==
    // OpenDrive::EnType::ODR_TYPE_ROAD)))
    if ((pos_info1.road->getSuccessor(1)) &&
        (pos_info1.road->getSuccessorLink() == NULL)) {
        OpenDrive::RoadHeader* myroad = pos_info1.road->getSuccessor(1);
        OpenDrive::JuncHeader* myJunction =
            reinterpret_cast<OpenDrive::JuncHeader*>(myroad->getJunction());
        OpenDrive::JuncLink* myJuncLink =
            reinterpret_cast<OpenDrive::JuncLink*>(myJunction->getFirstLink());
        int junctionId = myJunction->mId;
        int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
        int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
        int mConnectDir = 0;
        if (myJuncLink->mDir == 0) {
            mConnectDir = 10;
        } else {
            mConnectDir = 11;
        }
        OpenDrive::JuncLaneLink* myLaneLink = myJuncLink->getFirstLaneLink();
        // std::cout << " myroadid" <<pos_info1.roadId<<std::endl;
        // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
        // std::cout << " mConnectRoadId" <<mConnectRoadId<<std::endl;
        // std::cout << " mConnectLaneId"
        // <<myLaneLink->mConnectingLane->mId<<std::endl;
        if (mIncomRoadId == pos_info1.road->mId) {
            // a[0][0]=mConnectRoadId;
            a[0][0] = 10;
            // std::cout << " 1myroadid" <<pos_info1.roadId<<std::endl;
            // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
            if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                // a[0][1]=myLaneLink->mConnectingLane->mId;
                a[0][1] = mConnectDir;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    // std::cout << " 2myroadid" <<pos_info1.roadId<<std::endl;
                    // std::cout << " myroadid"
                    // <<pos_info1.lane->mId<<std::endl;
                    if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                        // a[0][1]=myLaneLink->mConnectingLane->mId;
                        a[0][1] = mConnectDir;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
            std::cout << " mConnectRoadId" << mConnectRoadId << std::endl;
        } else if (mConnectRoadId == pos_info1.road->mId) {
            // a[0][0]=mIncomRoadId;
            a[0][0] = 11;
            // std::cout << " 3myroadid" <<pos_info1.roadId<<std::endl;
            // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
            if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                // a[0][1]=myLaneLink->mIncomingLane->mId;
                a[0][1] = mConnectDir;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    // std::cout << " 4myroadid" <<pos_info1.roadId<<std::endl;
                    // std::cout << " myroadid"
                    // <<pos_info1.lane->mId<<std::endl;
                    if (myLaneLink->mConnectingLane->mId ==
                        pos_info1.lane->mId) {
                        // a[0][1]=myLaneLink->mIncomingLane->mId;
                        a[0][1] = mConnectDir;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
            // std::cout << " mIncomRoadId" <<mIncomRoadId<<std::endl;
        }
        while (myJuncLink->getRight() != NULL) {
            OpenDrive::JuncLink* nextJuncLink =
                (OpenDrive::JuncLink*)myJuncLink->getRight();
            myJuncLink = nextJuncLink;
            OpenDrive::JuncLaneLink* myLaneLink =
                myJuncLink->getFirstLaneLink();
            int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
            int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
            // std::cout << " 2mIncomRoadId" <<mIncomRoadId<<std::endl;
            // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl;
            // // pos_info1.road=pos_info1.road->getH();
            // // const double&GeoCoord::getH();
            // // pos_info1.roadheading = pos_info1.road->getH();
            // // if (mIncomRoadId ==
            // pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
            if (mIncomRoadId == pos_info1.road->mId) {
                // std::cout << " 5myroadid" <<pos_info1.roadId<<std::endl;
                // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
                // std::cout << " myLaneLink->mIncomingLane->mId"
                // <<myLaneLink->mIncomingLane->mId<<std::endl;
                // a[ai][0]=mConnectRoadId;
                a[ai][0] = 10;
                if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                    // a[ai][1]=myLaneLink->mConnectingLane->mId;
                    a[ai][1] = mConnectDir;
                    // std::cout << " 5gxmyroadid" <<std::endl;
                } else {
                    std::cout << " 55myroadid" << std::endl;
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        // std::cout << " 566myLaneLink->mConnectingLane->mId:"
                        // <<std::endl; std::cout << "
                        // 566myLaneLink->mConnectingLane->mId:"
                        // <<myLaneLink->mConnectingLane->mId<<std::endl;
                        if (myLaneLink->mIncomingLane->mId ==
                            pos_info1.lane->mId) {
                            // a[ai][1]=myLaneLink->mConnectingLane->mId;
                            a[ai][1] = mConnectDir;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            // std::cout << " 56myroadid"
                            // <<pos_info1.roadId<<std::endl;
                            break;
                        }
                    }
                }
                ai = ai + 1;
                // std::cout << " 2mConnectRoadId" <<mConnectRoadId<<std::endl;
            }
            // else if (mConnectRoadId ==
            // pos_info1.road->mId)&&((pos_info1.roadheading-heading1)<(pi/4))
            else if (mConnectRoadId == pos_info1.road->mId) {
                // std::cout << " 6myroadid" <<pos_info1.roadId<<std::endl;
                // std::cout << " myroadid" <<pos_info1.lane->mId<<std::endl;
                // a[ai][0]=mIncomRoadId;
                a[ai][0] = 11;
                if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                    a[ai][1] = mConnectDir;
                    // a[ai][1]=myLaneLink->mIncomingLane->mId;
                } else {
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        if (myLaneLink->mConnectingLane->mId ==
                            pos_info1.lane->mId) {
                            // a[ai][1]=myLaneLink->mIncomingLane->mId;
                            a[ai][1] = mConnectDir;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            break;
                        }
                    }
                }
                ai = ai + 1;
                // std::cout << " 2mIncomRoadId" <<mConnectRoadId<<std::endl;
            }
            if (myJuncLink->getRight() == NULL) {
                break;
            }
        }
        if (a[2][1]) {
            // std::cout << " shaixuan" <<std::endl;
            OpenDrive::OdrManager myManager;
            OpenDrive::Position*  pos = myManager.createPosition();
            myManager.activatePosition(pos);
            OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
            myManager.setPos(myCoord);
            bool result = myManager.lane2inertial();
            pos_info1.roadheading = myCoord.getH();
            int aiMax = ai;
            for (ai = 0; ai <= aiMax; ai++) {
                if ((pos_info1.roadheading - heading1) < (pi / 6) &&
                    (a[ai][1])) {
                    a[bi][0] = a[ai][0];
                    a[bi][1] = a[ai][1];
                    bi = bi + 1;
                }
                // std::cout << " bi " <<bi<<std::endl;
            }
            // std::cout << " shaixuanjieshu "
            // <<pos_info1.roadheading<<std::endl;
        }
        ai = 0;
        bi = 0;
        // std::cout << " a" <<a<<std::endl;
    }
    return a;
}

// gx
int** getPosInfo::Preprocess(Point p1, float heading1)
// int* getPosInfo::Preprocess()
{
    int** a;
    a = (int**)malloc(10 * sizeof(int*));
    for (int ai = 0; ai < 10; ai++) {
        a[ai] = (int*)malloc(2 * sizeof(int));
    }
    int ai = 0;

    int bi = 0;

    float pi = 4 * atan(1);
    bool  isInRoad = getInertialPosInfo(p1);
    if (!isInRoad) {
        a[0][0] = -1;
        return a;
    }
    posInfo pos_info1 = m_posInfo;
    pos_info1.roadId = pos_info1.road->mId;
    pos_info1.laneId = pos_info1.lane->mId;
    // // roadheading = myCoord.getH();
    // std::cout << "Preprocess enable :pos_info1.roadId is "
    // <<pos_info1.roadId<<std::endl; std::cout << "Preprocess enable
    // :pos_info1.laneId is " <<pos_info1.laneId<<std::endl;
    // // //if junction
    // // if (((pos_info1.roadlength-pos_info1.trackS<20)
    // // &&(pos_info1.road->getSuccessor())
    // // &&(pos_info1.road->getSuccessorLink()->getElemType() ==
    // OpenDrive::EnType::ODR_TYPE_ROAD)))
    if ((pos_info1.road->getSuccessor(1)) &&
        (pos_info1.road->getSuccessorLink() == NULL)) {
        OpenDrive::RoadHeader* myroad = pos_info1.road->getSuccessor(1);
        OpenDrive::JuncHeader* myJunction =
            reinterpret_cast<OpenDrive::JuncHeader*>(myroad->getJunction());
        OpenDrive::JuncLink* myJuncLink =
            reinterpret_cast<OpenDrive::JuncLink*>(myJunction->getFirstLink());
        int junctionId = myJunction->mId;
        int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
        int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
        OpenDrive::JuncLaneLink* myLaneLink = myJuncLink->getFirstLaneLink();
        if (mIncomRoadId == pos_info1.road->mId) {
            a[0][0] = mConnectRoadId;
            std::cout << " 1myroadid" << pos_info1.roadId << std::endl;
            std::cout << " myroadid" << pos_info1.lane->mId << std::endl;
            if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                a[0][1] = myLaneLink->mConnectingLane->mId;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                        a[0][1] = myLaneLink->mConnectingLane->mId;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
            std::cout << " mConnectRoadId" << mConnectRoadId << std::endl;
        } else if (mConnectRoadId == pos_info1.road->mId) {
            a[0][0] = mIncomRoadId;
            if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                a[0][1] = myLaneLink->mIncomingLane->mId;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    if (myLaneLink->mConnectingLane->mId ==
                        pos_info1.lane->mId) {
                        a[0][1] = myLaneLink->mIncomingLane->mId;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
        }
        while (myJuncLink->getRight() != NULL) {
            OpenDrive::JuncLink* nextJuncLink =
                (OpenDrive::JuncLink*)myJuncLink->getRight();
            myJuncLink = nextJuncLink;
            OpenDrive::JuncLaneLink* myLaneLink =
                myJuncLink->getFirstLaneLink();
            int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
            int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
            if (mIncomRoadId == pos_info1.road->mId) {
                a[ai][0] = mConnectRoadId;
                if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                    a[ai][1] = myLaneLink->mConnectingLane->mId;
                } else {
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        if (myLaneLink->mIncomingLane->mId ==
                            pos_info1.lane->mId) {
                            a[ai][1] = myLaneLink->mConnectingLane->mId;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            break;
                        }
                    }
                }
                ai = ai + 1;
            }
            else if (mConnectRoadId == pos_info1.road->mId) {
                a[ai][0] = mIncomRoadId;
                if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                    a[ai][1] = myLaneLink->mIncomingLane->mId;
                } else {
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        if (myLaneLink->mConnectingLane->mId ==
                            pos_info1.lane->mId) {
                            a[ai][1] = myLaneLink->mIncomingLane->mId;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            break;
                        }
                    }
                }
                ai = ai + 1;
            }
            if (myJuncLink->getRight() == NULL) {
                break;
            }
        }
        if (a[2][1]) {
            std::cout << " shaixuan" << std::endl;
            OpenDrive::OdrManager myManager;
            OpenDrive::Position*  pos = myManager.createPosition();
            myManager.activatePosition(pos);
            OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
            myManager.setPos(myCoord);
            bool result = myManager.lane2inertial();
            pos_info1.roadheading = myCoord.getH();
            int aiMax = ai;
            for (ai = 0; ai <= aiMax; ai++) {
                if ((pos_info1.roadheading - heading1) < (pi / 6) &&
                    (a[ai][1])) {
                    a[bi][0] = a[ai][0];
                    a[bi][1] = a[ai][1];
                    bi = bi + 1;
                }
            }
            std::cout << " bi " << bi - 1 << std::endl;
            std::cout << " shaixuanjieshu: " << pos_info1.roadheading
                      << std::endl;
        }
        ai = 0;
        bi = 0;
        // std::cout << " a" <<a<<std::endl;
    }
    return a;
}

// gx
int** getPosInfo::JuncLinkRoad(Point p1, float heading1)
// int* getPosInfo::Preprocess()
{
    int** a;
    a = (int**)malloc(10 * sizeof(int*));
    for (int ai = 0; ai < 10; ai++) {
        a[ai] = (int*)malloc(2 * sizeof(int));
    }
    int ai = 0;

    int bi = 0;

    float pi = 4 * atan(1);
    bool  isInRoad = getInertialPosInfo(p1);
    if (!isInRoad) {
        a[0][0] = -1;
        return a;
    }
    posInfo pos_info1 = m_posInfo;
    pos_info1.roadId = pos_info1.road->mId;
    pos_info1.laneId = pos_info1.lane->mId;
    if ((m_manager.getRoadHeader()->getJunctionNo() != -1)) {
        OpenDrive::RoadHeader* myroad = pos_info1.road;
        OpenDrive::JuncHeader* myJunction =
            reinterpret_cast<OpenDrive::JuncHeader*>(myroad->getJunction());
        OpenDrive::JuncLink* myJuncLink =
            reinterpret_cast<OpenDrive::JuncLink*>(myJunction->getFirstLink());
        int junctionId = myJunction->mId;
        int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
        int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
        OpenDrive::JuncLaneLink* myLaneLink = myJuncLink->getFirstLaneLink();
        if (mIncomRoadId == pos_info1.road->mId) {
            a[0][0] = mConnectRoadId;
            if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                a[0][1] = myLaneLink->mConnectingLane->mId;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                        a[0][1] = myLaneLink->mConnectingLane->mId;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
        } else if (mConnectRoadId == pos_info1.road->mId) {
            a[0][0] = mIncomRoadId;
            if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                a[0][1] = myLaneLink->mIncomingLane->mId;
            } else {
                while (myLaneLink->getRight() != NULL) {
                    OpenDrive::JuncLaneLink* nextJuncLaneLink =
                        (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                    myLaneLink = nextJuncLaneLink;
                    if (myLaneLink->mConnectingLane->mId ==
                        pos_info1.lane->mId) {
                        a[0][1] = myLaneLink->mIncomingLane->mId;
                    }
                    if (myLaneLink->getRight() == NULL) {
                        break;
                    }
                }
            }
            ai = ai + 1;
        }
        while (myJuncLink->getRight() != NULL) {
            OpenDrive::JuncLink* nextJuncLink =
                (OpenDrive::JuncLink*)myJuncLink->getRight();
            myJuncLink = nextJuncLink;
            OpenDrive::JuncLaneLink* myLaneLink =
                myJuncLink->getFirstLaneLink();
            int mIncomRoadId = myJuncLink->mIncomingRoad->mId;
            int mConnectRoadId = myJuncLink->mConnectingRoad->mId;
            if (mIncomRoadId == pos_info1.road->mId) {
                a[ai][0] = mConnectRoadId;
                if (myLaneLink->mIncomingLane->mId == pos_info1.lane->mId) {
                    a[ai][1] = myLaneLink->mConnectingLane->mId;
                } else {
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        if (myLaneLink->mIncomingLane->mId ==
                            pos_info1.lane->mId) {
                            a[ai][1] = myLaneLink->mConnectingLane->mId;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            break;
                        }
                    }
                }
                ai = ai + 1;
            } else if (mConnectRoadId == pos_info1.road->mId) {
                a[ai][0] = mIncomRoadId;
                if (myLaneLink->mConnectingLane->mId == pos_info1.lane->mId) {
                    a[ai][1] = myLaneLink->mIncomingLane->mId;
                } else {
                    while (myLaneLink->getRight() != NULL) {
                        OpenDrive::JuncLaneLink* nextJuncLaneLink =
                            (OpenDrive::JuncLaneLink*)myLaneLink->getRight();
                        myLaneLink = nextJuncLaneLink;
                        if (myLaneLink->mConnectingLane->mId ==
                            pos_info1.lane->mId) {
                            a[ai][1] = myLaneLink->mIncomingLane->mId;
                        }
                        if (myLaneLink->getRight() == NULL) {
                            break;
                        }
                    }
                }
                ai = ai + 1;
            }
            if (myJuncLink->getRight() == NULL) {
                break;
            }
        }
        if (a[2][1]) {
            std::cout << " shaixuan" << std::endl;
            OpenDrive::OdrManager myManager;
            OpenDrive::Position*  pos = myManager.createPosition();
            myManager.activatePosition(pos);
            OpenDrive::GeoCoord myCoord(p1.x, p1.y, p1.z);
            myManager.setPos(myCoord);
            bool result = myManager.lane2inertial();
            pos_info1.roadheading = myCoord.getH();
            int aiMax = ai;
            for (ai = 0; ai <= aiMax; ai++) {
                if ((pos_info1.roadheading - heading1) < (pi / 6) &&
                    (a[ai][1])) {
                    a[bi][0] = a[ai][0];
                    a[bi][1] = a[ai][1];
                    bi = bi + 1;
                }
            }
            std::cout << " bi " << bi - 1 << std::endl;
            std::cout << " shaixuanjieshu: " << pos_info1.roadheading
                      << std::endl;
        }
        ai = 0;
        bi = 0;
    }
    return a;
}

// gx
int getPosInfo::isInBackLane(Point p1, Point p2, float heading1, float* length)
{
    std::cout << "isInBackLane enable 1" << std::endl;
    bool isInRoad = getInertialPosInfo(p1);
    if (!isInRoad)
        return 0;
    posInfo pos_info1 = m_posInfo;
    isInRoad = getInertialPosInfo(p2);
    posInfo pos_info2 = m_posInfo;
    if (!isInRoad)
        return 0;
    int    isInFrontLaneFlag = 0;
    int    isInBackLaneFlag = 0;
    int    LengthSumMax = 150;
    double lengthSum = 0;
    // check if in same road
    if (pos_info1.roadId == pos_info2.roadId) {
        std::cout << "isInSameRoad " << std::endl;
        // check if in same laneSection
        if (pos_info1.sectionS == pos_info2.sectionS) {
            std::cout << "isInSameSec " << std::endl;
            // the same road and the same lane
            if (pos_info1.laneId == pos_info2.laneId) {
                std::cout << "isInSamelane " << std::endl;
                if (pos_info1.trackS > pos_info2.trackS) {
                    int isInBackLaneFlag = 1;
                    std::cout
                        << "in same lane isInBackLaneFlag :" << isInBackLaneFlag
                        << endl;
                    lengthSum = pos_info1.trackS - pos_info2.trackS;
                    (*length) = lengthSum;
                    return 11;
                }
            }
        } else if (pos_info1.sectionS > pos_info2.sectionS) {
            std::cout << "isInbacksec " << std::endl;
            bool endwhileFlag = 0;
            while ((pos_info1.roadId == pos_info2.roadId) &&
                   (pos_info1.sectionS < pos_info2.sectionS)) {
                // pos_info2.sectionS=pos_info2.sectionS->getNextSections();
                OpenDrive::LaneSection* secInfo2 = pos_info2.section;
                OpenDrive::LaneSection* nextSecInfo2 =
                    (OpenDrive::LaneSection*)secInfo2->getRight();
                pos_info2.section = nextSecInfo2;
                // OpenDrive::LaneLink*
                // laneInfo2=pos_info2.lane->getSuccessorLink();
                // pos_info2.laneId = laneInfo2-> mLaneId;
                pos_info2.laneId = pos_info2.lane->getSuccessor()->mId;
                if (pos_info1.section == pos_info2.section) {
                    if (pos_info1.laneId == pos_info2.laneId) {
                        lengthSum = pos_info1.trackS - pos_info2.trackS;
                        (*length) = lengthSum;
                        return 11;
                        // 2 is at the back of 1;
                        int isInBackLaneFlag = 1;
                        std::cout << "in same road not one section  "
                                     "isInBackLaneFlag :"
                                  << isInBackLaneFlag << endl;
                    } else {
                        return -1;
                        // the same road but not the same lane
                    }
                    endwhileFlag = 1;
                }
                if (endwhileFlag == 1) {
                    endwhileFlag = 0;
                    break;
                }
            }
        }
    }
    // check if not in same road ,if in the Back road;
    std::cout << "isbackroad " << std::endl;
    bool    endwhileFlag = 0;
    bool    nextroadI = 0;
    posInfo pos_info1old = pos_info1;
    posInfo pos_info1old1 = pos_info1;
    pos_info1.roadlength = 0;
    int GetRoadSuccessFlag = 10;
    // lengthSum = fabs(lengthSum) +  fabs(pos_info1.roadlength);
    while ((pos_info1.roadId != pos_info2.roadId) &&
           (lengthSum < LengthSumMax)) {
        // std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
        // pos_info1.roadlength = pos_info1.road->mLength;
        std::cout << "isBackroad " << std::endl;
        std::cout << "1pos_info1.roadId " << pos_info1.roadId << std::endl;
        std::cout << "2roadlength " << pos_info1.roadlength << std::endl;
        if (((pos_info1.road->getSuccessorLink() != NULL) &&
             GetRoadSuccessFlag == 11) ||
            ((pos_info1.road->getPredecessorLink() != NULL) &&
             GetRoadSuccessFlag == 10)) {
            std::cout << "3getSuccessorLink " << std::endl;
            std::cout << "4pos_info1.sectionS  " << pos_info1.sectionS
                      << std::endl;
            std::cout << "5pos_info1.lane  " << pos_info1.laneId << std::endl;
            bool nextRoadFlag = 0;
            // if have next sec
            {
                // if ((pos_info1old1.road->getSuccessorDir() == 0) &&
                // (nextroadI == 1))
                if ((GetRoadSuccessFlag == 11) && (nextroadI == 1)) {
                    std::cout << "6getSuccessorLink " << std::endl;
                    // OpenDrive::LaneLink*
                    // laneInfo2=pos_info2.lane->getSuccessorLink();
                    pos_info1old1.road = pos_info1old.road;
                    pos_info1old.road = pos_info1.road;
                    if (pos_info1old1.road->getSuccessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                    } else {
                        GetRoadSuccessFlag = 10;
                    }
                    pos_info1.road = pos_info1.road->getSuccessor();
                    pos_info1.roadId = pos_info1.road->mId;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getSuccessorLink()) {
                        pos_info1.lane = pos_info1.lane->getSuccessor();
                        pos_info1.laneId = pos_info1.lane->mId;
                        std::cout << "7pos_info1.laneId " << pos_info1.laneId
                                  << std::endl;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    if ((pos_info1old.road->getSuccessorDir() == 0) &&
                        (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "88sectionS  " << pos_info1.sectionS
                                  << std::endl;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "8pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.lane = pos_info1.lane->getSuccessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                                std::cout << "9pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    } else if ((pos_info1old.road->getSuccessorDir() == 1) &&
                               (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "10pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.lane =
                                    pos_info1.lane->getPredecessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                                std::cout << "11pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    }
                }

                // if ((pos_info1old1.road->getSuccessorDir() == 1) &&
                // (nextroadI == 1))
                if ((GetRoadSuccessFlag == 10) && (nextroadI == 1)) {
                    std::cout << "12getSuccessorLink " << std::endl;
                    // OpenDrive::LaneLink*
                    // laneInfo2=pos_info2.lane->getSuccessorLink();
                    pos_info1old1.road = pos_info1old.road;
                    pos_info1old.road = pos_info1.road;
                    if (pos_info1old1.road->getPredecessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                    } else {
                        GetRoadSuccessFlag = 10;
                    }
                    pos_info1.road = pos_info1.road->getPredecessor();
                    pos_info1.roadId = pos_info1.road->mId;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getPredecessorLink()) {
                        pos_info1.laneId =
                            pos_info1.lane->getPredecessor()->mId;
                        std::cout << "13pos_info1.laneId " << pos_info1.laneId
                                  << std::endl;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    if ((pos_info1old.road->getSuccessorDir() == 0) &&
                        (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "88sectionS  " << pos_info1.sectionS
                                  << std::endl;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 11;
                                // int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "14pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.laneId =
                                    pos_info1.lane->getSuccessor()->mId;
                                std::cout << "15pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.lane == pos_info2.lane) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    // int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    } else if ((pos_info1old.road->getSuccessorDir() == 1) &&
                               (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "88sectionS  " << pos_info1.sectionS
                                  << std::endl;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 11;
                                int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "16pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.laneId =
                                    pos_info1.lane->getPredecessor()->mId;
                                std::cout << "17pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.lane == pos_info2.lane) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    // int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    }
                } else if (nextroadI == 0) {
                    std::cout << "18pos_info1.roadId " << pos_info1.roadId
                              << std::endl;
                    while (pos_info1.section->getLeft()) {
                        OpenDrive::LaneSection* secInfo1 = pos_info1.section;
                        OpenDrive::LaneSection* nextSecInfo1 =
                            (OpenDrive::LaneSection*)secInfo1->getLeft();
                        pos_info1.section = nextSecInfo1;
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "19pos_info1.sectionS  "
                                  << pos_info1.sectionS << std::endl;
                        lengthSum =
                            fabs(lengthSum) + fabs(pos_info1.section->mS);
                        // pos_info1.sectionS = nextSecInfo1->mS;
                        if (pos_info1.lane->getPredecessorLink()) {
                            pos_info1.lane = pos_info1.lane->getPredecessor();
                            pos_info1.laneId = pos_info1.lane->mId;
                            std::cout << "20pos_info1.laneId "
                                      << pos_info1.laneId << std::endl;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                            break;
                        }
                    }
                    pos_info1old.road = pos_info1.road;
                    pos_info1.road = pos_info1.road->getPredecessor();
                    pos_info1.roadId = pos_info1.road->mId;
                    std::cout << "2611pos_info1.roadId " << pos_info1.roadId
                              << std::endl;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getPredecessorLink()) {
                        pos_info1.lane = pos_info1.lane->getPredecessor();
                        pos_info1.laneId = pos_info1.lane->mId;
                        std::cout << "2111pos_info1.laneId " << pos_info1.laneId
                                  << std::endl;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    lengthSum = pos_info1.trackS;
                    (*length) = lengthSum;
                    if (pos_info1old.road->getPredecessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "2211pos_info1.sectionS  "
                                  << pos_info1.sectionS << std::endl;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 11;
                                // int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "22pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum =
                                fabs(lengthSum) + fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.lane = pos_info1.lane->getSuccessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                                std::cout << "23pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    }
                    if (pos_info1old.road->getPredecessorDir() == 1) {
                        GetRoadSuccessFlag = 10;
                        // OpenDrive::LaneSection *Roadinfo1= pos_info1old.road;
                        // OpenDrive::LaneSection
                        // *firstsec=(OpenDrive::LaneSection
                        // *)Roadinfo1->getFirstLaneSection(); pos_info1.section
                        // = pos_info1.road->getFirstLaneSection();
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        std::cout << "2411pos_info1.sectionS  "
                                  << pos_info1.sectionS << std::endl;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << lengthSum << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info2.laneId << std::endl;
                            std::cout
                                << "30pos_info1.roadId == pos_info2.roadId "
                                << pos_info1.laneId << std::endl;
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 11;
                                // int isInFrontLaneFlag = 1;
                                std::cout << "31in not one road not one lane "
                                             "isInFrontLaneFlag :"
                                          << isInFrontLaneFlag << endl;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                            std::cout << "32lengthSum " << lengthSum
                                      << std::endl;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "24pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum =
                                fabs(lengthSum) + fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.lane =
                                    pos_info1.lane->getPredecessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                                std::cout << "25pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    // int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    }
                    nextroadI = 1;
                    std::cout << "26pos_info1.roadId " << pos_info1.roadId
                              << std::endl;
                    std::cout << "27pos_info1.sectionS  " << pos_info1.sectionS
                              << std::endl;
                    std::cout << "28pos_info1.laneId " << pos_info1.laneId
                              << std::endl;
                }
                std::cout << "29lengthSum " << lengthSum << std::endl;
            }
            if ((pos_info1.roadId == pos_info2.roadId) &&
                (pos_info1.section == pos_info2.section)) {
                std::cout << "30pos_info1.roadId == pos_info2.roadId "
                          << pos_info2.lane << std::endl;
                std::cout << "30pos_info1.roadId == pos_info2.roadId "
                          << pos_info1.lane << std::endl;
                if (pos_info1.laneId == pos_info2.laneId) {
                    return 11;
                    int isInFrontLaneFlag = 1;
                    std::cout
                        << "31in not one road not one lane isInFrontLaneFlag :"
                        << isInFrontLaneFlag << endl;
                } else {
                    return 0;
                }
                endwhileFlag = 1;
                std::cout << "32lengthSum " << lengthSum << std::endl;
            } else if (lengthSum >= LengthSumMax) {
                endwhileFlag = 1;
                isInBackLaneFlag = 1;
                std::cout << "33lengthSum " << lengthSum << std::endl;
            }
            // if (isInBackLaneFlag==1)
            // {
            //   std::cout << "34lengthSum " << lengthSum << std::endl;
            //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1, p2);
            //   if (isInFrontLaneFlag==10)
            //   {
            //     return 10;
            //   }
            //   endwhileFlag=1;
            // }
            if (endwhileFlag == 1) {
                std::cout << "35lengthSum " << lengthSum << std::endl;
                endwhileFlag = 0;
                (*length) = lengthSum;
                lengthSum = 0;
                break;
            }
        }
        // if Successor Type is Junction
        if ((pos_info1.road->getPredecessor(1)) &&
            (pos_info1.road->getPredecessorLink() == NULL)) {
            // bool isInRoad = getInertialPosInfo(p1);
            std::cout << "36ODR_TYPE_JUNCTION " << std::endl;
            OpenDrive::OdrManager Manager;
            OpenDrive::Position*  pos1 = Manager.createPosition();
            Manager.activatePosition(pos1);
            Manager.setLanePos(
                pos_info1.roadId, pos_info1.laneId, pos_info1.trackS);
            bool   result = Manager.lane2inertial();
            double p1x = Manager.getInertialPos().getX();
            double p1y = Manager.getInertialPos().getY();
            double p1z = Manager.getInertialPos().getZ();
            int**  RoadConnectId =
                getPosInfo::Preprocess(Point(p1x, p1y, p1z), heading1);
            int** JunDir =
                getPosInfo::GetJuncDir(Point(p1x, p1y, p1z), heading1);
            if (RoadConnectId[0][0] == -1) {
                return -1;
                break;
            }
            std::cout << "yuchuliwanbi " << std::endl;
            // read the connectRoadId/incomingId
            std::cout << "jieguochangdu "
                      << 2 * sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0])
                      << std::endl;

            for (int RoadConnectI = 0;
                 RoadConnectI <
                 (2 * sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0]));
                 RoadConnectI++) {
                std::cout << "nextroadid" << pos_info1.roadId << std::endl;
                std::cout << "nextlaneid" << pos_info1.laneId << std::endl;
                std::cout << "for  RoadConnectI" << std::endl;
                int nextroadid = RoadConnectId[RoadConnectI][0];
                int nextlaneid = RoadConnectId[RoadConnectI][1];
                int nextRoadDir = JunDir[RoadConnectI][1];
                int IncomRoadFlag = JunDir[RoadConnectI][0];
                std::cout << "nextroadid" << nextroadid << std::endl;
                std::cout << "nextlaneid" << nextlaneid << std::endl;
                OpenDrive::OdrManager myManager;
                OpenDrive::Position*  pos = myManager.createPosition();
                myManager.activatePosition(pos);
                myManager.setLanePos(nextroadid, nextlaneid, 0.01);
                bool result = myManager.lane2inertial();
                std::cout << "for  Connectroad1:" << result << std::endl;
                OpenDrive::RoadHeader* Connectroad = myManager.getRoadHeader();
                OpenDrive::Lane*       Connectlane = myManager.getLane();
                int Connectroadid = myManager.getRoadHeader()->mId;
                std::cout << "for  Connectroad" << Connectroadid << std::endl;
                std::cout << " 111mConnectRoadId" << Connectroad->mId
                          << std::endl;
                if (IncomRoadFlag == nextRoadDir) {
                    std::cout << " pos_info1.roadId: " << std::endl;
                    // pos_info1.road= Connectroad;
                    // pos_info1.roadId=pos_info1.road->mId;
                    // std::cout << " pos_info1.roadId: " <<  pos_info1.roadId
                    // << std::endl;

                    if ((IncomRoadFlag == 10) && (nextroadI == 1)) {
                        std::cout << "6getSuccessorLink " << std::endl;

                        if (pos_info1old1.road->getSuccessorDir() == 0) {
                            GetRoadSuccessFlag = 11;
                        } else {
                            GetRoadSuccessFlag = 10;
                        }
                        pos_info1old1.road = pos_info1old.road;
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = Connectroad;
                        pos_info1.roadId = pos_info1.road->mId;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (Connectlane) {
                            pos_info1.lane = Connectlane;
                            pos_info1.laneId = pos_info1.lane->mId;
                            std::cout << "7pos_info1.laneId "
                                      << pos_info1.laneId << std::endl;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        if ((nextRoadDir == 10) && (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "88sectionS  " << pos_info1.sectionS
                                      << std::endl;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info2.laneId << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info1.laneId << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                std::cout << "8pos_info1.sectionS  "
                                          << pos_info1.sectionS << std::endl;
                                lengthSum = fabs(lengthSum) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getSuccessor()->mId;
                                    std::cout << "9pos_info1.laneId "
                                              << pos_info1.laneId << std::endl;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    std::cout << "30pos_info1.roadId == "
                                                 "pos_info2.roadId "
                                              << lengthSum << std::endl;
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 11;
                                        int isInFrontLaneFlag = 1;
                                        std::cout
                                            << "31in not one road not one lane "
                                               "isInFrontLaneFlag :"
                                            << isInFrontLaneFlag << endl;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    std::cout << "32lengthSum " << lengthSum
                                              << std::endl;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        } else if ((pos_info1.road->getPredecessorDir() == 0) &&
                                   (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getLastLaneSection();
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info2.laneId << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info1.laneId << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                            }
                            while (pos_info1.section->getLeft()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getLeft();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                std::cout << "10pos_info1.sectionS  "
                                          << pos_info1.sectionS << std::endl;
                                lengthSum = fabs(lengthSum) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getPredecessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getPredecessor()->mId;
                                    std::cout << "11pos_info1.laneId "
                                              << pos_info1.laneId << std::endl;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    std::cout << "30pos_info1.roadId == "
                                                 "pos_info2.roadId "
                                              << lengthSum << std::endl;
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum = lengthSum +
                                                    pos_info2.roadlength -
                                                    pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 11;
                                        int isInFrontLaneFlag = 1;
                                        std::cout
                                            << "31in not one road not one lane "
                                               "isInFrontLaneFlag :"
                                            << isInFrontLaneFlag << endl;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    std::cout << "32lengthSum " << lengthSum
                                              << std::endl;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                    }
                    if ((IncomRoadFlag == 11) && (nextroadI == 1)) {
                        std::cout << "12getSuccessorLink " << std::endl;
                        // OpenDrive::LaneLink*
                        // laneInfo2=pos_info2.lane->getSuccessorLink();
                        if (pos_info1old1.road->getPredecessorDir() == 0) {
                            GetRoadSuccessFlag = 11;
                        } else {
                            GetRoadSuccessFlag = 10;
                        }
                        pos_info1old1.road = pos_info1old.road;
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = pos_info1.road->getPredecessor();
                        pos_info1.roadId = pos_info1.road->mId;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (pos_info1.lane->getPredecessorLink()) {
                            pos_info1.laneId =
                                pos_info1.lane->getPredecessor()->mId;
                            std::cout << "13pos_info1.laneId "
                                      << pos_info1.laneId << std::endl;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        if ((pos_info1.road->getPredecessorDir() == 1) &&
                            (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "88sectionS  " << pos_info1.sectionS
                                      << std::endl;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info2.laneId << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info1.laneId << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                std::cout << "14pos_info1.sectionS  "
                                          << pos_info1.sectionS << std::endl;
                                lengthSum = fabs(lengthSum) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getSuccessor()->mId;
                                    std::cout << "15pos_info1.laneId "
                                              << pos_info1.laneId << std::endl;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    std::cout << "30pos_info1.roadId == "
                                                 "pos_info2.roadId "
                                              << lengthSum << std::endl;
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 11;
                                        int isInFrontLaneFlag = 1;
                                        std::cout
                                            << "31in not one road not one lane "
                                               "isInFrontLaneFlag :"
                                            << isInFrontLaneFlag << endl;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    std::cout << "32lengthSum " << lengthSum
                                              << std::endl;
                                    break;
                                }
                            }
                        }
                        lengthSum = lengthSum + pos_info1.roadlength;
                        (*length) = lengthSum;
                    } else if (nextroadI == 0) {
                        std::cout << "118pos_info1.roadId " << pos_info1.roadId
                                  << std::endl;
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "119pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            lengthSum =
                                fabs(lengthSum) + fabs(pos_info1.section->mS);
                            // pos_info1.sectionS = nextSecInfo1->mS;
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.lane =
                                    pos_info1.lane->getPredecessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                                std::cout << "120pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                                break;
                            }
                        }
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = Connectroad;
                        pos_info1.roadId = pos_info1.road->mId;
                        std::cout << "2611pos_info1.roadId " << pos_info1.roadId
                                  << std::endl;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (Connectlane) {
                            pos_info1.lane = Connectlane;
                            pos_info1.laneId = pos_info1.lane->mId;
                            std::cout << "2111pos_info1.laneId "
                                      << pos_info1.laneId << std::endl;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        lengthSum = pos_info1.trackS;
                        (*length) = lengthSum;
                        if (IncomRoadFlag == 10) {
                            GetRoadSuccessFlag = 11;
                            std::cout << "1111xiangtongfangxiang  "
                                      << std::endl;
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "2211pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout << "30lengthSum " << lengthSum
                                          << std::endl;
                                std::cout << "30pos_info2.laneId "
                                          << pos_info2.laneId << std::endl;
                                std::cout << "30pos_info1.laneId "
                                          << pos_info1.laneId << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    // int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                std::cout << "22pos_info1.sectionS  "
                                          << pos_info1.sectionS << std::endl;
                                lengthSum = fabs(lengthSum) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.lane =
                                        pos_info1.lane->getSuccessor();
                                    pos_info1.laneId = pos_info1.lane->mId;
                                    std::cout << "23pos_info1.laneId "
                                              << pos_info1.laneId << std::endl;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    std::cout << "30pos_info1.roadId == "
                                                 "pos_info2.roadId "
                                              << lengthSum << std::endl;
                                    if (pos_info1.laneId == pos_info2.laneId) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 11;
                                        // int isInFrontLaneFlag = 1;
                                        std::cout
                                            << "31in not one road not one lane "
                                               "isInFrontLaneFlag :"
                                            << isInFrontLaneFlag << endl;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    std::cout << "32lengthSum " << lengthSum
                                              << std::endl;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                        if (IncomRoadFlag == 11) {
                            // OpenDrive::LaneSection *Roadinfo1=
                            // pos_info1old.road; OpenDrive::LaneSection
                            // *firstsec=(OpenDrive::LaneSection
                            // *)Roadinfo1->getFirstLaneSection();
                            // pos_info1.section =
                            // pos_info1.road->getFirstLaneSection();
                            GetRoadSuccessFlag = 10;
                            std::cout << "1111butongtongfangxiang  "
                                      << std::endl;
                            pos_info1.section =
                                pos_info1.road->getLastLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            std::cout << "2411pos_info1.sectionS  "
                                      << pos_info1.sectionS << std::endl;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << lengthSum << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info2.laneId << std::endl;
                                std::cout
                                    << "30pos_info1.roadId == pos_info2.roadId "
                                    << pos_info1.laneId << std::endl;
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 11;
                                    int isInFrontLaneFlag = 1;
                                    std::cout << "31in not one road not one "
                                                 "lane isInFrontLaneFlag :"
                                              << isInFrontLaneFlag << endl;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                std::cout << "32lengthSum " << lengthSum
                                          << std::endl;
                            }
                            while (pos_info1.section->getLeft()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getLeft();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                std::cout << "24pos_info1.sectionS  "
                                          << pos_info1.sectionS << std::endl;
                                lengthSum = fabs(lengthSum) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getPredecessorLink()) {
                                    pos_info1.lane =
                                        pos_info1.lane->getPredecessor();
                                    pos_info1.laneId = pos_info1.lane->mId;
                                    std::cout << "25pos_info1.laneId "
                                              << pos_info1.laneId << std::endl;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    std::cout << "30pos_info1.roadId == "
                                                 "pos_info2.roadId "
                                              << lengthSum << std::endl;
                                    if (pos_info1.laneId == pos_info2.laneId) {
                                        lengthSum = lengthSum +
                                                    pos_info2.roadlength -
                                                    pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 11;
                                        int isInFrontLaneFlag = 1;
                                        std::cout
                                            << "31in not one road not one lane "
                                               "isInFrontLaneFlag :"
                                            << isInFrontLaneFlag << endl;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    std::cout << "32lengthSum " << lengthSum
                                              << std::endl;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                        nextroadI = 1;
                        std::cout << "26pos_info1.roadId " << pos_info1.roadId
                                  << std::endl;
                        std::cout << "27pos_info1.sectionS  "
                                  << pos_info1.sectionS << std::endl;
                        std::cout << "28pos_info1.laneId " << pos_info1.laneId
                                  << std::endl;
                    }
                    std::cout << "29lengthSum " << lengthSum << std::endl;
                    if ((pos_info1.roadId == pos_info2.roadId) &&
                        (pos_info1.section == pos_info2.section)) {
                        std::cout << "30pos_info1.roadId == pos_info2.roadId "
                                  << pos_info2.lane << std::endl;
                        std::cout << "30pos_info1.roadId == pos_info2.roadId "
                                  << pos_info1.lane << std::endl;
                        if (pos_info1.laneId == pos_info2.laneId) {
                            return 11;
                            int isInFrontLaneFlag = 1;
                            std::cout << "31in not one road not one lane "
                                         "isInFrontLaneFlag :"
                                      << isInFrontLaneFlag << endl;
                        } else {
                            return 0;
                        }
                        endwhileFlag = 1;
                        std::cout << "32lengthSum " << lengthSum << std::endl;
                    } else if (lengthSum >= 150) {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                        std::cout << "33lengthSum " << lengthSum << std::endl;
                    }
                    // if (isInBackLaneFlag==1)
                    // {
                    //   std::cout << "34lengthSum " << lengthSum << std::endl;
                    //   int isInFrontLaneFlag = getPosInfo::isInBackLane(p1,
                    //   p2); if (isInFrontLaneFlag==11)
                    //   {
                    //     return 11;
                    //   }
                    //   endwhileFlag=1;
                    // }
                    if (endwhileFlag == 1) {
                        std::cout << "35lengthSum " << lengthSum << std::endl;
                        endwhileFlag = 0;
                        (*length) = lengthSum;
                        lengthSum = 0;
                        break;
                    }
                }
                std::cout << "jieguo: " << nextroadid << std::endl;
                std::cout << "jieguo: " << nextlaneid << std::endl;
            }
            break;
            // need to fixed
        }
        std::cout << "isInFrontLaneFlag " << isInFrontLaneFlag << std::endl;
    }
    return 0;
}

// gx
int getPosInfo::isInFrontLane(Point p1, Point p2, float heading1, float* length)
{
    bool isInRoad = getInertialPosInfo(p1);
    if (!isInRoad)
        return 0;
    posInfo pos_info1 = m_posInfo;
    isInRoad = getInertialPosInfo(p2);
    posInfo pos_info2 = m_posInfo;
    if (!isInRoad)
        return 0;
    int    isInFrontLaneFlag = 0;
    int    isInBackLaneFlag = 0;
    int    LengthSumMax = 150;
    double lengthSum = 0;

    // check if in same road
    if (pos_info1.roadId == pos_info2.roadId) {
        // check if in same laneSection
        if (pos_info1.sectionS == pos_info2.sectionS) {
            // the same road and the same lane
            if (pos_info1.laneId == pos_info2.laneId) {
                if (pos_info1.trackS < pos_info2.trackS) {
                    lengthSum = pos_info2.trackS - pos_info1.trackS;
                    (*length) = lengthSum;
                    int isInFrontLaneFlag = 1;
                    return 10;
                }
            }
        } else if (pos_info1.sectionS < pos_info2.sectionS) {
            bool endwhileFlag = 0;
            while ((pos_info1.roadId == pos_info2.roadId) &&
                   (pos_info1.sectionS < pos_info2.sectionS)) {
                OpenDrive::LaneSection* secInfo1 = pos_info1.section;
                OpenDrive::LaneSection* nextSecInfo1 =
                    (OpenDrive::LaneSection*)secInfo1->getRight();
                pos_info1.section = nextSecInfo1;
                pos_info1.laneId = pos_info1.lane->getSuccessor()->mId;
                if (pos_info1.section == pos_info2.section) {
                    if (pos_info1.laneId == pos_info2.laneId) {
                        lengthSum = pos_info2.trackS - pos_info1.trackS;
                        (*length) = lengthSum;
                        return 10;
                        // 2 is at the front of 1;
                        int isInFrontLaneFlag = 1;
                    } else {
                        return -1;
                        // the same road but not the same lane
                    }
                    endwhileFlag = 1;
                }
                if (endwhileFlag == 1) {
                    endwhileFlag = 0;
                    break;
                }
            }
        }
    }
    // check if not in same road ,if in the front road;
    bool    endwhileFlag = 0;
    bool    nextroadI = 0;
    posInfo pos_info1old = pos_info1;
    posInfo pos_info1old1 = pos_info1;
    pos_info1.roadlength = 0;
    int GetRoadSuccessFlag = 11;

    // pos_info1old.road = pos_info1.road;
    while ((pos_info1.roadId != pos_info2.roadId) &&
           (lengthSum < LengthSumMax)) {
        if (((pos_info1.road->getSuccessorLink() != NULL) &&
             GetRoadSuccessFlag == 11) ||
            ((pos_info1.road->getPredecessorLink() != NULL) &&
             GetRoadSuccessFlag == 10)) {
            bool nextRoadFlag = 0;
            // if have next sec
            {
                if ((GetRoadSuccessFlag == 11) && (nextroadI == 1)) {
                    pos_info1old1.road = pos_info1old.road;
                    pos_info1old.road = pos_info1.road;
                    if (pos_info1old1.road->getSuccessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                    } else {
                        GetRoadSuccessFlag = 10;
                    }
                    pos_info1.roadId = pos_info1.road->mId;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getSuccessorLink()) {
                        pos_info1.lane = pos_info1.lane->getSuccessor();
                        pos_info1.laneId = pos_info1.lane->mId;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    if ((pos_info1old.road->getSuccessorDir() == 0) &&
                        (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.lane = pos_info1.lane->getSuccessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.road->mLength;
                        (*length) = lengthSum;
                    } else if ((pos_info1old.road->getSuccessorDir() == 1) &&
                               (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.lane =
                                    pos_info1.lane->getPredecessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.road->mLength;
                        (*length) = lengthSum;
                    }
                }
                if ((GetRoadSuccessFlag == 10) && (nextroadI == 1)) {
                    pos_info1old1.road = pos_info1old.road;
                    pos_info1old.road = pos_info1.road;
                    if (pos_info1old1.road->getPredecessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                    } else {
                        GetRoadSuccessFlag = 10;
                    }
                    pos_info1.road = pos_info1.road->getPredecessor();
                    pos_info1.roadId = pos_info1.road->mId;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getPredecessorLink()) {
                        pos_info1.laneId =
                            pos_info1.lane->getPredecessor()->mId;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    if ((pos_info1old.road->getSuccessorDir() == 0) &&
                        (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.laneId =
                                    pos_info1.lane->getSuccessor()->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.lane == pos_info2.lane) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                    } else if ((pos_info1old.road->getSuccessorDir() == 1) &&
                               (nextroadI == 1)) {
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.laneId =
                                    pos_info1.lane->getPredecessor()->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.lane == pos_info2.lane) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                    }
                } else if (nextroadI == 0) {
                    while (pos_info1.section->getRight()) {
                        OpenDrive::LaneSection* secInfo1 = pos_info1.section;
                        OpenDrive::LaneSection* nextSecInfo1 =
                            (OpenDrive::LaneSection*)secInfo1->getRight();
                        pos_info1.section = nextSecInfo1;
                        pos_info1.sectionS = pos_info1.section->mS;
                        if (pos_info1.lane->getSuccessorLink()) {
                            pos_info1.lane = pos_info1.lane->getSuccessor();
                            pos_info1.laneId = pos_info1.lane->mId;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                            break;
                        }
                    }
                    pos_info1old.road = pos_info1.road;
                    pos_info1.road = pos_info1.road->getSuccessor();
                    pos_info1.roadId = pos_info1.road->mId;
                    pos_info1.roadlength = pos_info1.road->mLength;
                    if (pos_info1.lane->getSuccessorLink()) {
                        pos_info1.lane = pos_info1.lane->getSuccessor();
                        pos_info1.laneId = pos_info1.lane->mId;
                    } else {
                        endwhileFlag = 1;
                        isInBackLaneFlag = 1;
                    }
                    lengthSum = pos_info1.roadlength - pos_info1.trackS;
                    if (pos_info1old.road->getSuccessorDir() == 0) {
                        GetRoadSuccessFlag = 11;
                        pos_info1.section =
                            pos_info1.road->getFirstLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.lane = pos_info1.lane->getSuccessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.road->mLength;
                    }
                    if (pos_info1old.road->getSuccessorDir() == 1) {
                        GetRoadSuccessFlag = 10;
                        pos_info1.section =
                            pos_info1.road->getLastLaneSection();
                        pos_info1.sectionS = pos_info1.section->mS;
                        if ((pos_info1.roadId == pos_info2.roadId) &&
                            (pos_info1.section == pos_info2.section)) {
                            if (pos_info1.laneId == pos_info2.laneId) {
                                lengthSum = lengthSum + pos_info2.roadlength -
                                            pos_info2.trackS;
                                (*length) = lengthSum;
                                return 10;
                                int isInFrontLaneFlag = 1;
                            } else {
                                return 0;
                            }
                            endwhileFlag = 1;
                        }
                        while (pos_info1.section->getLeft()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getLeft();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getPredecessorLink()) {
                                pos_info1.lane =
                                    pos_info1.lane->getPredecessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                            }
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                                break;
                            }
                        }
                        lengthSum = lengthSum + pos_info1.road->mLength;
                    }
                    nextroadI = 1;
                }
            }
            if ((pos_info1.roadId == pos_info2.roadId) &&
                (pos_info1.section == pos_info2.section)) {
                if (pos_info1.laneId == pos_info2.laneId) {
                    return 10;
                    int isInFrontLaneFlag = 1;
                } else {
                    return 0;
                }
                endwhileFlag = 1;
            } else if (lengthSum >= LengthSumMax) {
                endwhileFlag = 1;
            }
            if (isInBackLaneFlag == 1) {
                endwhileFlag = 1;
            }
            if (endwhileFlag == 1) {
                lengthSum = 0;
                break;
            }
        }
        // if Successor Type is Junction
        if (((pos_info1.road->getSuccessor(1)) &&
             (pos_info1.road->getSuccessorLink() == NULL) &&
             GetRoadSuccessFlag == 11) ||
            ((pos_info1.road->getPredecessor(1)) &&
             (pos_info1.road->getPredecessorLink() == NULL) &&
             GetRoadSuccessFlag == 10)) {
            OpenDrive::OdrManager Manager;
            OpenDrive::Position*  pos1 = Manager.createPosition();
            Manager.activatePosition(pos1);
            Manager.setLanePos(
                pos_info1.roadId, pos_info1.laneId, pos_info1.trackS);
            bool   result = Manager.lane2inertial();
            double p1x = Manager.getInertialPos().getX();
            double p1y = Manager.getInertialPos().getY();
            double p1z = Manager.getInertialPos().getZ();
            int**  RoadConnectId =
                getPosInfo::Preprocess(Point(p1x, p1y, p1z), heading1);
            int** JunDir =
                getPosInfo::GetJuncDir(Point(p1x, p1y, p1z), heading1);
            if (RoadConnectId[0][0] == -1) {
                return -1;
                break;
            }
            for (int RoadConnectI = 0;
                 RoadConnectI <
                 (2 * sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0]));
                 RoadConnectI++) {
                int nextroadid = RoadConnectId[RoadConnectI][0];
                int nextlaneid = RoadConnectId[RoadConnectI][1];
                int nextRoadDir = JunDir[RoadConnectI][1];
                int IncomRoadFlag = JunDir[RoadConnectI][0];
                OpenDrive::OdrManager myManager;
                OpenDrive::Position*  pos = myManager.createPosition();
                myManager.activatePosition(pos);
                myManager.setLanePos(nextroadid, nextlaneid, 0.01);
                bool result = myManager.lane2inertial();
                std::cout << "for  Connectroad1:" << result << std::endl;
                OpenDrive::RoadHeader* Connectroad = myManager.getRoadHeader();
                OpenDrive::Lane*       Connectlane = myManager.getLane();
                int Connectroadid = myManager.getRoadHeader()->mId;
                std::cout << "for  Connectroad" << Connectroadid << std::endl;
                // pos_info1.road为Incoming&contactpoint为0，或者pos_info1.road为Connecting&contactpoint为1
                if (IncomRoadFlag == nextRoadDir) {
                    if ((IncomRoadFlag == 10) && (nextroadI == 1)) {
                        pos_info1old1.road = pos_info1old.road;
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = Connectroad;
                        pos_info1.roadId = pos_info1.road->mId;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (pos_info1old1.road->getSuccessorDir() == 0) {
                            GetRoadSuccessFlag = 11;
                        } else {
                            GetRoadSuccessFlag = 10;
                        }
                        if (Connectlane) {
                            pos_info1.lane = Connectlane;
                            pos_info1.laneId = pos_info1.lane->mId;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        if ((nextRoadDir == 10) && (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                lengthSum = fabs(pos_info1.trackS) +
                                            fabs(pos_info1.section->mS);
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getSuccessor()->mId;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 10;
                                        int isInFrontLaneFlag = 1;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        } else if ((pos_info1.road->getPredecessorDir() == 0) &&
                                   (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getLastLaneSection();
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                            }
                            while (pos_info1.section->getLeft()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getLeft();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                lengthSum = fabs(pos_info1.trackS) +
                                            fabs(pos_info1.section->mS);
                                if (pos_info1.lane->getPredecessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getPredecessor()->mId;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum = lengthSum +
                                                    pos_info2.roadlength -
                                                    pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 10;
                                        int isInFrontLaneFlag = 1;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                    }
                    if ((IncomRoadFlag == 11) && (nextroadI == 1)) {
                        pos_info1old1.road = pos_info1old.road;
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = pos_info1.road->getPredecessor();
                        pos_info1.roadId = pos_info1.road->mId;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (pos_info1.lane->getPredecessorLink()) {
                            pos_info1.laneId =
                                pos_info1.lane->getPredecessor()->mId;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        if ((pos_info1.road->getPredecessorDir() == 1) &&
                            (nextroadI == 1)) {
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                lengthSum = fabs(pos_info1.trackS) +
                                            fabs(pos_info1.section->mS);
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.laneId =
                                        pos_info1.lane->getSuccessor()->mId;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    if (pos_info1.lane == pos_info2.lane) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 10;
                                        int isInFrontLaneFlag = 1;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                    } else if (nextroadI == 0) {
                        while (pos_info1.section->getRight()) {
                            OpenDrive::LaneSection* secInfo1 =
                                pos_info1.section;
                            OpenDrive::LaneSection* nextSecInfo1 =
                                (OpenDrive::LaneSection*)secInfo1->getRight();
                            pos_info1.section = nextSecInfo1;
                            pos_info1.sectionS = pos_info1.section->mS;
                            lengthSum = fabs(pos_info1.trackS) +
                                        fabs(pos_info1.section->mS);
                            if (pos_info1.lane->getSuccessorLink()) {
                                pos_info1.lane = pos_info1.lane->getSuccessor();
                                pos_info1.laneId = pos_info1.lane->mId;
                            } else {
                                endwhileFlag = 1;
                                isInBackLaneFlag = 1;
                                break;
                            }
                        }
                        pos_info1old.road = pos_info1.road;
                        pos_info1.road = Connectroad;
                        pos_info1.roadId = pos_info1.road->mId;
                        pos_info1.roadlength = pos_info1.road->mLength;
                        if (Connectlane) {
                            pos_info1.lane = Connectlane;
                            pos_info1.laneId = pos_info1.lane->mId;
                        } else {
                            endwhileFlag = 1;
                            isInBackLaneFlag = 1;
                        }
                        lengthSum = pos_info1.roadlength - pos_info1.trackS;
                        (*length) = lengthSum;
                        if (IncomRoadFlag == 10) {
                            GetRoadSuccessFlag = 11;
                            pos_info1.section =
                                pos_info1.road->getFirstLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum + pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                            }
                            while (pos_info1.section->getRight()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getRight();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                lengthSum = fabs(pos_info1.trackS) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getSuccessorLink()) {
                                    pos_info1.lane =
                                        pos_info1.lane->getSuccessor();
                                    pos_info1.laneId = pos_info1.lane->mId;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    if (pos_info1.laneId == pos_info2.laneId) {
                                        lengthSum =
                                            lengthSum + pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 10;
                                        int isInFrontLaneFlag = 1;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                        if (IncomRoadFlag == 11) {
                            GetRoadSuccessFlag = 10;
                            pos_info1.section =
                                pos_info1.road->getLastLaneSection();
                            pos_info1.sectionS = pos_info1.section->mS;
                            if ((pos_info1.roadId == pos_info2.roadId) &&
                                (pos_info1.section == pos_info2.section)) {
                                if (pos_info1.laneId == pos_info2.laneId) {
                                    lengthSum = lengthSum +
                                                pos_info2.roadlength -
                                                pos_info2.trackS;
                                    (*length) = lengthSum;
                                    return 10;
                                    int isInFrontLaneFlag = 1;
                                } else {
                                    return 0;
                                }
                                endwhileFlag = 1;
                            }
                            while (pos_info1.section->getLeft()) {
                                OpenDrive::LaneSection* secInfo1 =
                                    pos_info1.section;
                                OpenDrive::LaneSection* nextSecInfo1 =
                                    (OpenDrive::LaneSection*)
                                        secInfo1->getLeft();
                                pos_info1.section = nextSecInfo1;
                                pos_info1.sectionS = pos_info1.section->mS;
                                lengthSum = fabs(pos_info1.trackS) +
                                            fabs(pos_info1.section->mS);
                                // pos_info1.sectionS = nextSecInfo1->mS;
                                if (pos_info1.lane->getPredecessorLink()) {
                                    pos_info1.lane =
                                        pos_info1.lane->getPredecessor();
                                    pos_info1.laneId = pos_info1.lane->mId;
                                } else {
                                    endwhileFlag = 1;
                                    isInBackLaneFlag = 1;
                                }
                                if ((pos_info1.roadId == pos_info2.roadId) &&
                                    (pos_info1.section == pos_info2.section)) {
                                    if (pos_info1.laneId == pos_info2.laneId) {
                                        lengthSum = lengthSum +
                                                    pos_info2.roadlength -
                                                    pos_info2.trackS;
                                        (*length) = lengthSum;
                                        return 10;
                                        int isInFrontLaneFlag = 1;
                                    } else {
                                        return 0;
                                    }
                                    endwhileFlag = 1;
                                    break;
                                }
                            }
                            lengthSum = lengthSum + pos_info1.roadlength;
                            (*length) = lengthSum;
                        }
                        nextroadI = 1;
                    }
                    if ((pos_info1.roadId == pos_info2.roadId) &&
                        (pos_info1.section == pos_info2.section)) {
                        if (pos_info1.laneId == pos_info2.laneId) {
                            return 10;
                            int isInFrontLaneFlag = 1;
                        } else {
                            return 0;
                        }
                        endwhileFlag = 1;
                    } else if (lengthSum >= LengthSumMax) {
                        endwhileFlag = 1;
                    }
                    if (endwhileFlag == 1) {
                        lengthSum = 0;
                        break;
                    }
                }
                if (RoadConnectI >=
                    (2 * sizeof(RoadConnectId) / sizeof(RoadConnectId[0][0]))) {
                    break;
                }
            }
            // need to fixed
            if ((pos_info1.roadId == pos_info2.roadId) &&
                (pos_info1.section == pos_info2.section)) {
                if (pos_info1.laneId == pos_info2.laneId) {
                    return 10;
                    int isInFrontLaneFlag = 1;
                } else {
                    return 0;
                }
                endwhileFlag = 1;
            } else if (lengthSum >= LengthSumMax) {
                endwhileFlag = 1;
                isInBackLaneFlag = 1;
            }
            if (isInBackLaneFlag == 1) {
                endwhileFlag = 1;
            }
            if (endwhileFlag == 1) {
                endwhileFlag = 0;
                (*length) = lengthSum;
                lengthSum = 0;
                break;
            }
        }
    }
    return 0;
}

// gx
bool getPosInfo::isFrontCar(Point  p1,
                            Point  p2,
                            float  heading1,
                            float  heading2,
                            float* length)
{
    bool isInRoad = getInertialPosInfo(p1);
    if (!isInRoad)
        return 0;
    isInRoad = getInertialPosInfo(p2);
    if (!isInRoad)
        return 0;
    std::cout << "is front car enable " << std::endl;
    float pi = 4 * atan(1);
    // std::cout << "isInFrontOrBackLaneFlag " << isInFrontOrBackLaneFlag <<
    // std::endl; std::cout << "heading1 " << heading1 << std::endl;
    if ((((pi / 2) > heading1) && (heading1 > 0)) ||
        (((-pi / 2) < heading1) && (heading1 < 0))) {
        int isInFrontOrBackLaneFlag =
            getPosInfo::isInFrontLane(p1, p2, heading1, length);
        std::cout << "1isInFrontOrBackLaneFlag:" << isInFrontOrBackLaneFlag
                  << std::endl;
        if (isInFrontOrBackLaneFlag == 10) {
            std::cout << "isFrontCar " << std::endl;
            return 1;
        }
    } else if ((((pi / 2) <= heading1) && (heading1 <= pi)) ||
               (((-pi / 2) >= heading1) && (heading1 >= (-pi)))) {
        int isInFrontOrBackLaneFlag =
            getPosInfo::isInBackLane(p1, p2, heading1, length);
        std::cout << "2isInFrontOrBackLaneFlag:" << isInFrontOrBackLaneFlag
                  << std::endl;
        if (isInFrontOrBackLaneFlag == 10) {
            std::cout << "isFrontCar " << std::endl;
            return 1;
        }
    }
    std::cout << "is front car end " << std::endl;
    return 0;
}

// gx
bool getPosInfo::isBackCar(Point  p1,
                           Point  p2,
                           float  heading1,
                           float  heading2,
                           float* length)
{
    bool isInRoad = getInertialPosInfo(p1);
    if (!isInRoad)
        return 0;
    isInRoad = getInertialPosInfo(p2);
    if (!isInRoad)
        return 0;
    float pi = 4 * atan(1);
    if (((pi / 2) > heading1 > 0) || ((-pi / 2) < heading1 < 0)) {
        int isInFrontOrBackLaneFlag =
            getPosInfo::isInBackLane(p1, p2, heading1, length);
        if (isInFrontOrBackLaneFlag == 11) {
            std::cout << "isBackCar " << std::endl;
            return 1;
        }
    } else if ((((pi / 2) <= heading1) && (heading1 <= pi)) ||
               (((-pi / 2) >= heading1) && (heading1 >= (-pi)))) {
        int isInFrontOrBackLaneFlag =
            getPosInfo::isInFrontLane(p1, p2, heading1, length);
        if (isInFrontOrBackLaneFlag == 11) {
            std::cout << "isBackCar " << std::endl;
            return 1;
        }
    }
    return 0;
}

// jm/hmx
bool getPosInfo::isOnRoadMark(const Point& p)
{
    OpenDrive::Position* pos = m_manager.createPosition();
    m_manager.activatePosition(pos);

    m_manager.setInertialPos(p.x, p.y, p.z);
    bool result = m_manager.inertial2lane();

    // TODO:it's better to get the width of roadmark from opendrive file or vtd
    const double wheelWidth = 0.2;
    if (result) {
        OpenDrive::Lane*       myLane = m_manager.getLane();
        OpenDrive::RoadHeader* header = m_manager.getRoadHeader();
        double                 laneWidth = m_manager.getLaneWidth();
        OpenDrive::LaneCoord   posLane = m_manager.getLanePos();
        double                 offsetFromLaneCenter = posLane.getOffset();
        // cout <<"laneOffset is:" <<offsetFromLaneCenter<< endl;

        double dist =
            laneWidth / 2 - fabs(offsetFromLaneCenter) - wheelWidth / 2;

        if (myLane->mId > 0)  // lane is at left of the road
        {
            if (offsetFromLaneCenter >=
                0)  // position is at left of the lane, then myLane->roadMark is
                    // the target roadMark
            {
                OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();

                if (myRoadMark == NULL) {
                    return false;
                }
                cout << "roadMarkType:" << myRoadMark->getType() << endl;

                if (dist < myRoadMark->getWidth() / 2) {

                    if (myRoadMark->getType() == 1 ||
                        myRoadMark->getType() == 3) {
                        cout << "ShiXian!" << endl;
                        return true;
                    } else {
                        cout << "XuXian!" << endl;
                        return false;
                    }
                } else {
                    cout << "too far from roadMark" << endl;
                    return false;
                }
            } else  // position is at right of the lane, then
                    // myRIGHTLane->roadMark is the target roadMark
            {
                OpenDrive::Lane* myRightLane =
                    (OpenDrive::Lane*)myLane->getRight();

                if (myRightLane == NULL)
                    return false;
                OpenDrive::RoadMark* myRoadMark =
                    myRightLane->getFirstRoadMark();
                cout << "roadMarkType:" << myRoadMark->getType() << endl;
                if (myRoadMark == NULL) {
                    return false;
                }
                if (dist < myRoadMark->getWidth() / 2) {
                    if (myRoadMark->getType() == 1 ||
                        myRoadMark->getType() == 3) {
                        cout << "ShiXian!" << endl;
                        return true;
                    } else {
                        cout << "XuXian!" << endl;
                        return false;
                    }
                } else {
                    cout << "too far from roadMark" << endl;
                    return false;
                }
            }
        } else  // lane is at right of the road
        {
            if (offsetFromLaneCenter >=
                0)  // position is at left of the lane, then
                    // myLEFTLane->roadMark is the target roadMark
            {
                OpenDrive::Lane* myLeftLane =
                    (OpenDrive::Lane*)myLane->getLeft();
                if (myLeftLane == NULL)
                    return false;
                OpenDrive::RoadMark* myRoadMark =
                    myLeftLane->getFirstRoadMark();
                cout << "roadMarkType:" << myRoadMark->getType() << endl;
                if (myRoadMark == NULL) {
                    return false;
                }
                if (dist < myRoadMark->getWidth() / 2) {
                    if (myRoadMark->getType() == 1 ||
                        myRoadMark->getType() == 3) {
                        cout << "ShiXian!" << endl;
                        return true;
                    } else {
                        cout << "XuXian!" << endl;
                        return false;
                    }
                } else {
                    cout << "too far from roadMark" << endl;
                    return false;
                }
            } else  // position is at right of the lane, then myLane->roadMark
                    // is the target roadMark
            {
                OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
                cout << "roadMarkType:" << myRoadMark->getType() << endl;
                if (myRoadMark == NULL) {
                    return false;
                }
                if (dist < myRoadMark->getWidth() / 2) {
                    if (myRoadMark->getType() == 1 ||
                        myRoadMark->getType() == 3) {
                        cout << "ShiXian!" << endl;
                        return true;
                    } else {
                        cout << "XuXian!" << endl;
                        return false;
                    }
                } else {
                    cout << "too far from roadMark" << endl;
                    return false;
                }
            }
        }

        // if(offsetFromLaneCenter >= 0 )  // position is at left of the lane
        // {
        //   if(myLane->mId > 0)           // lane is at left of the road, then
        //   myLane->roadMark is the target roadMark
        //   {
        //     OpenDrive::RoadMark* myRoadMark = myLane->getFirstRoadMark();
        //     if(myRoadMark->getType() == 1)
        //       return true;
        //     else
        //       return false;
        //   }
        //   else              // lane is at right of the road, then
        //   myLeftLane->roadMark is the target roadMark
        //   {
        //     OpenDrive::Lane* myLeftlane =
        //     (OpenDrive::Lane*)myLane->getLeft(); OpenDrive::RoadMark*
        //     myRoadMark = myLane->getFirstRoadMark(); if(myRoadMark->getType()
        //     == 1)
        //       return true;
        //     else
        //       return false;
        //   }
        // }
        // else                               // position is at left of the lane
        // {

        // }

        // cout <<"myLaneId is:" <<myLane->mId<< endl;
        // OpenDrive::Lane* myLeftlane = (OpenDrive::Lane*)myLane->getLeft();
        // cout <<"myLeftlaneId is:" << myLeftlane->mId<< endl;
    }
    return result;
}

// jm//gx20220305
bool getPosInfo::isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType,OpenDrive::EnAccessRestriction laneAccessRestrictType)
{
    getInertialPosInfo(p);
    if(m_posInfo.laneType == roadType)
    {
        return 1;
    }
    else 
    {
        OpenDrive::Node* mylaneNode = m_posInfo.lane;
        if (mylaneNode->getChild(45))
        {
            OpenDrive::LaneAccess* laneNodeChild=(OpenDrive::LaneAccess*) mylaneNode->getChild(45);
            unsigned int myLaneAccessRestrict=0;
            if (laneNodeChild->mRestriction)
            {
                myLaneAccessRestrict=laneNodeChild->mRestriction;
                if(myLaneAccessRestrict==laneAccessRestrictType)
                {
                    return 1;
                }
            }
        }
    }
    return 0;
}

// jm
bool getPosInfo::isInSpecialLaneType(Point p, OpenDrive::EnLaneType roadType)
{
    getInertialPosInfo(p);
    if(m_posInfo.laneType == roadType)
    {
        return 1;
    }
    return 0;
}

// jm
// TODO: the car is stopped, lastS == curS
int getPosInfo::drivingDirRelToRoad(double lastS, double curS)
{
    if (curS > lastS)
        return 1;
    else if (curS < lastS)
        return -1;
}

// jm /gx 20220302
double getPosInfo::findLimitedSpeed(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (isInRoad) {
        OpenDrive::RoadHeader* road = m_posInfo.road;
        auto                   speedSigns = getAllSpeedSignsOfRoad(road);
        int                    drivingDir = drivingDirRelToRoad();
        double                 curS = m_posInfo.trackS;

        // 1. current road has valid speed sign
        if (drivingDir == 1) {
            for (int i = speedSigns.size() - 1; i >= 0; --i) {
                auto         sign = speedSigns[i];
                double       signS = sign->mS;
                unsigned int signDir = sign->mDir;
                if (signDir == 0 && curS > signS) {
                    return sign->mValue;
                }
            }
        }
        if (drivingDir == -1) {
            for (int i = 0; i < speedSigns.size(); ++i) {
                auto         sign = speedSigns[i];
                double       signS = sign->mS;
                unsigned int signDir = sign->mDir;
                if (signDir == 1 && curS < signS) {
                    return sign->mValue;
                }
            }
        }

        // 2. current raod has no valid speed sign
        OpenDrive::RoadHeader* lastRoad = NULL;
        int                    lastRoadDir = -1;
        if (drivingDir == 1) {
            lastRoad = road->getPredecessor();
            lastRoadDir = (int)road->getPredecessorDir();
        } else {
            lastRoad = road->getSuccessor();
            lastRoadDir = (int)road->getSuccessorDir();
        }

        while (lastRoad) {
            speedSigns = getAllSpeedSignsOfRoad(lastRoad);
            if (lastRoadDir == 0) {
                for (int i = 0; i < speedSigns.size(); ++i) {
                    auto         sign = speedSigns[i];
                    unsigned int signDir = sign->mDir;
                    // contactPoint->start  orientation-> -
                    if (signDir == 1) {
                        return sign->mValue;
                    }
                }
            }

            if (lastRoadDir == 1) {
                for (int i = speedSigns.size() - 1; i >= 0; --i) {
                    auto         sign = speedSigns[i];
                    unsigned int signDir = sign->mDir;
                    // contactPoint->start  orientation-> -
                    if (signDir == 0) {
                        // std::cout << "forth." << std::endl;
                        return sign->mValue;
                    }
                }
            }

            if (lastRoadDir == 0) {
                auto roadS = lastRoad->getSuccessor();
                lastRoadDir = (int)lastRoad->getSuccessorDir();
                lastRoad = roadS;
            } else if (lastRoadDir == 1) {
                auto roadP = lastRoad->getPredecessor();
                lastRoadDir = (int)lastRoad->getPredecessorDir();
                lastRoad = roadP;
            }
        }

        // by default return the speed of current lane
        return m_posInfo.laneSpeed;
    } else {
        return -2;
    }
}

// gx
bool getPosInfo::isSameDirectionWithRoad(Point p, float heading1)
{
    getInertialPosInfo(p);
    float pi = atan(1) * 4;
    float CarHeading = m_posInfo.roadheading - heading1;
    // std::cout<<"m_posInfo.roadheading"<<m_posInfo.roadheading<<std::endl;
    if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0)) {
        return 1;
    }
    return 0;
}

// is Turning:gx
bool getPosInfo::isInSce2_46_7(Point p,float *StartHeading,float *EndHeading)
{
    bool   isInRoad = getInertialPosInfo(p);
    if (isInRoad)
    {
        double myLaneSecEnd;
        if (m_posInfo.section->getRight())
        {
            OpenDrive::LaneSection*  myLaneSecNext =(OpenDrive::LaneSection*) m_posInfo.section->getRight();
            myLaneSecEnd= myLaneSecNext->mS;
        }
        else 
        {
            double  slength = m_posInfo.roadlength;
            myLaneSecEnd=slength;
        }
        OpenDrive::OdrManager Manager;
        OpenDrive::Position*  pos1 = Manager.createPosition();
        Manager.activatePosition(pos1);
        Manager.setLanePos(m_posInfo.roadId, m_posInfo.laneId, m_posInfo.sectionS+0.00001);
        Manager.lane2inertial();
        double              p1x = Manager.getInertialPos().getX();
        double              p1y = Manager.getInertialPos().getY();
        double              p1z = Manager.getInertialPos().getZ();
        Point p1(p1x, p1y, p1z);
        bool isInRoad1 = getInertialPosInfo(p1);
        posInfo pos_info1 = m_posInfo;
        double  startH = pos_info1.roadheading;
        (*StartHeading)=startH ;
        OpenDrive::OdrManager myManager;
        OpenDrive::Position*  pos2 = myManager.createPosition();
        myManager.activatePosition(pos2);
        myManager.setLanePos(m_posInfo.roadId, pos_info1.lane->mId, myLaneSecEnd-0.00001);
        myManager.lane2inertial();
        double              p2x = myManager.getInertialPos().getX();
        double              p2y = myManager.getInertialPos().getY();
        double              p2z = myManager.getInertialPos().getZ();
        Point p2(p2x, p2y, p2z);
        bool isInRoad2 = getInertialPosInfo(p2);
        posInfo pos_info2 = m_posInfo;
        double  endH = pos_info2.roadheading;
        (*EndHeading)=endH ;
        float pi=atan(1)*4;
        std ::cout<<endH<<std ::endl;
        std ::cout<<startH<<std ::endl;
        if (startH != endH) {
            std ::cout<<endH<<std ::endl;
            std ::cout<<startH<<std ::endl;
            return 1;
        }
    }
    return 0;
}

// isTheSteepSlope:gx
bool getPosInfo::isInSce2_46_8(Point p, float heading1, float *MySlope,float *MySlopeHeight)
{
    bool isInRoad = getInertialPosInfo(p);
    bool alongRoadFlag = getPosInfo::isSameDirectionWithRoad(p, heading1);
    OpenDrive::Elevation* myElevation = m_posInfo.posElevation;
    double                elvLength = (m_posInfo.trackS) - (m_posInfo.ElvmS);
    double                elvHeight = m_posInfo.ElvmB * elvLength +
                       m_posInfo.ElvmC * elvLength * elvLength +
                       m_posInfo.ElvmD * elvLength * elvLength * elvLength;
    *MySlopeHeight=m_posInfo.ElvmA+m_posInfo.ElvmB * (m_posInfo.trackS) +
                       m_posInfo.ElvmC * (m_posInfo.trackS) * (m_posInfo.trackS) +
                       m_posInfo.ElvmD * (m_posInfo.trackS) * (m_posInfo.trackS) * (m_posInfo.trackS);
    int    hIcreaseFlag = 0;
    double slopeMin = 0.07;
    *MySlope=elvHeight / elvLength;
    if ((abs(elvHeight / elvLength)) > slopeMin) {
        float sChange = (elvLength) / 4;
        float hChange = m_posInfo.ElvmB * sChange +
                        m_posInfo.ElvmC * sChange * sChange +
                        m_posInfo.ElvmD * sChange * sChange * sChange;
        if (hChange > 0) {
            hIcreaseFlag = 1;
        } else if (hChange < 0) {
            hIcreaseFlag = 2;
        }
    }
    if ((alongRoadFlag == 1 && hIcreaseFlag == 2) ||
        (alongRoadFlag == 0 && hIcreaseFlag == 1)) {
        return 1;
    }
    return 0;
}
                                                                                                                                                                      
// isTopRam:gx
bool getPosInfo::isInSce2_59_2(Point p)
{
    getInertialPosInfo(p);
    if ((m_posInfo.ElvmA == 0) && (m_posInfo.ElvmB == 0) &&
        (m_posInfo.ElvmC == 0) && (m_posInfo.ElvmD == 0)) {
        return 0;
    } else {
        float sPos = m_posInfo.trackS - m_posInfo.ElvmS;
        float d2s = m_posInfo.ElvmB + 2 * m_posInfo.ElvmC * sPos;
        if (d2s < 0) {
            return 1;
        } else if ((d2s >= 0) && (d2s < 0.1)) {
            float sChangeLeft = (m_posInfo.trackS - m_posInfo.ElvmS) / 10;
            float sChangeRight = (m_posInfo.ElvmSEnd - m_posInfo.trackS) / 10;
            int   sPosI = 1;
            float d2sRight =
                2 * m_posInfo.ElvmC +
                6 * m_posInfo.ElvmD * (sPos + sChangeRight * sPosI);
            float d2sLeft = 2 * m_posInfo.ElvmC +
                            6 * m_posInfo.ElvmD * (sPos - sChangeLeft * sPosI);
            if (((d2sRight < 0) && (d2sLeft <= 0)) ||
                ((d2sRight = 0) && (d2sLeft < 0))) {
                return 1;
            }
            while ((d2sRight == 0) && (d2sLeft == 0)) {
                sPosI = sPosI + 1;
                d2sRight = 2 * m_posInfo.ElvmC +
                           6 * m_posInfo.ElvmD * (sPos + sChangeRight * sPosI);
                d2sLeft = 2 * m_posInfo.ElvmC +
                          6 * m_posInfo.ElvmD * (sPos - sChangeLeft * sPosI);
                if (((d2sRight < 0.01) && (d2sLeft <= 0)) ||
                    ((d2sRight = 0) && (d2sLeft < 0.01))) {
                    sPosI = 1;
                    return 1;
                    break;
                } else if ((d2sRight > 0.1) || (d2sLeft > 0.1)) {
                    sPosI = 1;
                    break;
                } else if (sPosI > 10) {

                    break;
                }
            }
        }
    }

    return 0;
    // OpenDrive::Elevation*  lastElevation = (OpenDrive::Elevation*
    // )myElevation->getLeft(); std::cout << "isTopRam start11111." <<
    // std::endl; if(m_posInfo.ElvmB != (lastElevation->mB))
    // {
    //   float sChangeLast= (lastElevation->mSEnd-lastElevation->mS)/10;
    //   float sChangeL=sChangeLast*9;
    //   float
    //   hChangeLast=lastElevation->mB*sChangeL+lastElevation->mC*sChangeL*sChangeL+lastElevation->mD*sChangeL*sChangeL*sChangeL;
    //   float sChange= (m_posInfo.ElvmSEnd-m_posInfo.ElvmS)/10;
    //   float
    //   hChange=m_posInfo.ElvmB*sChange+m_posInfo.ElvmC*sChange*sChange+m_posInfo.ElvmD*sChange*sChange*sChange;
    //   if (hChangeLast>0)
    //   {
    //     return 1;
    //   }
    //   else if (hChangeLast==0)
    //   {
    //     if (hChange<0)
    //     {
    //       return 1;
    //     }
    //   }
    //   else
    //   {
    //     if (hChange<=0)
    //     {
    //       return 1;
    //     }
    //   }
    // }
}

// isNarrowRoad:gx
bool getPosInfo::isInSec2_46_4(Point p,float *MyRoadWidth)
{
    // std::cout << "isInSec2_46_4 start" << std::endl;
    bool isInRoad = getInertialPosInfo(p);
    if (isInRoad) {
        double           laneWidth = m_manager.getLaneWidth();
        double           RoadWidth=0;
        double           RoadWidthMax = 5;
        bool             endwhileFlag = 0;
        posInfo          posInfo1 = m_posInfo;
        posInfo          posInfo2 = m_posInfo;
        OpenDrive::Lane* laneInfonext = posInfo1.lane;
        OpenDrive::Lane* laneInfolast = posInfo2.lane;
        if (m_posInfo.laneType==(OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING))
        {
            RoadWidth=laneWidth;
            *MyRoadWidth=RoadWidth;
        }
        while (((OpenDrive::Lane*)laneInfonext->getRight())) {
            OpenDrive::Lane* nextLaneInfo1 =
                (OpenDrive::Lane*)laneInfonext->getRight();
            posInfo1.lane = nextLaneInfo1;
            posInfo1.laneId = posInfo1.lane->mId;
            laneInfonext = nextLaneInfo1;
            if (laneInfonext->mType ==
                (OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)) {
                m_manager.setLanePos(
                    posInfo1.roadId, posInfo1.laneId, posInfo1.trackS);
                m_manager.lane2inertial();
                posInfo1.laneWidth = m_manager.getLaneWidth();
                RoadWidth = RoadWidth + posInfo1.laneWidth;
                *MyRoadWidth=RoadWidth;
            }
            if ((RoadWidth > RoadWidthMax)) {
                return 0;
                break;
            } else if ((OpenDrive::Lane*)laneInfonext->getRight() == NULL) {
                endwhileFlag = 1;
                break;
            }
        }
        if (endwhileFlag == 1) {
            while ((OpenDrive::Lane*)laneInfolast->getLeft()) {
                OpenDrive::Lane* lastLaneInfo1 =
                    (OpenDrive::Lane*)laneInfolast->getLeft();
                posInfo2.lane = lastLaneInfo1;
                posInfo2.laneId = posInfo2.lane->mId;
                laneInfolast = lastLaneInfo1;
                if (laneInfolast->mType == (OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)) 
                {
                    m_manager.setLanePos(
                        posInfo2.roadId, posInfo2.laneId, posInfo2.trackS);
                    m_manager.lane2inertial();
                    posInfo2.laneWidth = m_manager.getLaneWidth();
                    RoadWidth = RoadWidth + posInfo2.laneWidth;
                     *MyRoadWidth=RoadWidth;
                }
                if (RoadWidth > RoadWidthMax) {
                    return 0;
                    break;
                }
                if ((OpenDrive::Lane*)laneInfolast->getLeft() == NULL) {
                    break;
                }
            }
        }
        if((RoadWidth <= RoadWidthMax)&&(RoadWidth!=0)) {
            return 1;
        }
    }
    return 0;
}

// isNarrowBridge:gx
bool getPosInfo::isInSec2_46_5(Point p,float *MyBridgeWidth)
{
    bool isInRoad = getInertialPosInfo(p);
    bool isOnBridgeFlag = m_manager.onBridge();
    if (isOnBridgeFlag) {
        double           laneWidth = m_manager.getLaneWidth();
        double           BridgeWidth=0;
        double           RoadWidthMax = 5;
        bool             endwhileFlag = 0;
        posInfo          posInfo1 = m_posInfo;
        posInfo          posInfo2 = m_posInfo;
        OpenDrive::Lane* laneInfonext = posInfo1.lane;
        OpenDrive::Lane* laneInfolast = posInfo2.lane;
        if (m_posInfo.laneType==(OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING))
        {
            BridgeWidth=laneWidth;
            *MyBridgeWidth=BridgeWidth;
        }
        while (((OpenDrive::Lane*)laneInfonext->getRight())) {
            OpenDrive::Lane* nextLaneInfo1 =
                (OpenDrive::Lane*)laneInfonext->getRight();
            posInfo1.lane = nextLaneInfo1;
            posInfo1.laneId = posInfo1.lane->mId;
            laneInfonext = nextLaneInfo1;
            if (laneInfonext->mType ==
                (OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)) {
                m_manager.setLanePos(
                    posInfo1.roadId, posInfo1.laneId, posInfo1.trackS);
                m_manager.lane2inertial();
                posInfo1.laneWidth = m_manager.getLaneWidth();
                BridgeWidth = BridgeWidth + posInfo1.laneWidth;
                *MyBridgeWidth=BridgeWidth;
            }
            if ((BridgeWidth > RoadWidthMax)) {
                return 0;
                break;
            } else if ((OpenDrive::Lane*)laneInfonext->getRight() == NULL) {
                endwhileFlag = 1;
                break;
            }
        }
        if (endwhileFlag == 1) {
            while ((OpenDrive::Lane*)laneInfolast->getLeft()) {
                OpenDrive::Lane* lastLaneInfo1 =
                    (OpenDrive::Lane*)laneInfolast->getLeft();
                posInfo2.lane = lastLaneInfo1;
                posInfo2.laneId = posInfo2.lane->mId;
                laneInfolast = lastLaneInfo1;
                if (laneInfolast->mType == (OpenDrive::EnLaneType::ODR_LANE_TYPE_DRIVING)) 
                {
                    m_manager.setLanePos(
                        posInfo2.roadId, posInfo2.laneId, posInfo2.trackS);
                    m_manager.lane2inertial();
                    posInfo2.laneWidth = m_manager.getLaneWidth();
                    BridgeWidth = BridgeWidth + posInfo2.laneWidth;
                    *MyBridgeWidth=BridgeWidth;
                }
                if (BridgeWidth > RoadWidthMax) {
                    return 0;
                    break;
                }
                if ((OpenDrive::Lane*)laneInfolast->getLeft() == NULL) {
                    break;
                }
            }
        }
        if((BridgeWidth <= RoadWidthMax)&&(BridgeWidth!=0)) {
            return 1;
        }
    }
    return 0;
}

// isSideWalk:gx
bool getPosInfo::isInSec1_47_1(Point p, float heading1, float* SideWalkDis)
{
    bool isInRoad = getInertialPosInfo(p);
    bool alongRoadFlag = getPosInfo::isSameDirectionWithRoad(p, heading1);
    int  SideWalkDisMax = 5;
    // if sidewalk is at front within SideWalkDis meter
    if (m_posInfo.laneType == OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK) {
        (*SideWalkDis) = 0;
        return 1;
    } else if (alongRoadFlag) {
        if (m_posInfo.roadlength - m_posInfo.trackS - SideWalkDisMax > 0) {
            while (m_posInfo.section->getRight()) {
                OpenDrive::LaneSection* secInfo1 = m_posInfo.section;
                OpenDrive::LaneSection* nextSecInfo1 =
                    (OpenDrive::LaneSection*)secInfo1->getRight();
                m_posInfo.section = nextSecInfo1;
                m_posInfo.sectionS = m_posInfo.section->mS;
                if (m_posInfo.sectionS - m_posInfo.trackS - SideWalkDisMax >
                    0) {
                    break;
                } else if (m_posInfo.lane->getSuccessorLink()) {
                    m_posInfo.lane = m_posInfo.lane->getSuccessor();
                    m_posInfo.laneId = m_posInfo.lane->mId;
                    if (m_posInfo.laneType ==
                        OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK) {
                        (*SideWalkDis) =
                            m_posInfo.roadlength - m_posInfo.trackS;
                        return 1;
                    }
                    std::cout << "20pos_info1.laneId " << m_posInfo.laneId
                              << std::endl;
                } else {
                    break;
                }
            }
        }
    } else {
        if (m_posInfo.trackS - SideWalkDisMax > 0) {
            while (m_posInfo.section->getLeft()) {
                OpenDrive::LaneSection* secInfo1 = m_posInfo.section;
                OpenDrive::LaneSection* nextSecInfo1 =
                    (OpenDrive::LaneSection*)secInfo1->getLeft();
                m_posInfo.section = nextSecInfo1;
                m_posInfo.sectionS = m_posInfo.section->mS;
                if (m_posInfo.sectionS - m_posInfo.trackS - SideWalkDisMax >
                    0) {
                    break;
                } else if (m_posInfo.lane->getSuccessorLink()) {
                    m_posInfo.lane = m_posInfo.lane->getSuccessor();
                    m_posInfo.laneId = m_posInfo.lane->mId;
                    if (m_posInfo.laneType ==
                        OpenDrive::EnLaneType::ODR_LANE_TYPE_SIDEWALK) {
                        (*SideWalkDis) = m_posInfo.trackS;
                        return 1;
                    }
                    std::cout << "20pos_info1.laneId " << m_posInfo.laneId
                              << std::endl;
                } else {
                    break;
                }
            }
        }
    }
    return 0;
}

// TODO isInRail:gx 20220127
// can not test
bool getPosInfo::isInSec2_46_2(Point p)
{
    bool isInRail = getPosInfo::isInSpecialLaneType(
        p, OpenDrive::EnLaneType::ODR_LANE_TYPE_RAIL);
    if (isInRail) {
        return 1;
    }
    return 0;
}

// TODO isTurnBack:gx 20220127
// need scenario to test
// bool getPosInfo::isInSec2_46_6(Point p1,
//                                Point p2,
//                                float heading1,
//                                float heading2,
//                                int*  RoadID,float*  Roadheading)
// {
//     bool    isInRoad = getInertialPosInfo(p1);
//     posInfo pos_info1 = m_posInfo;
//     getInertialPosInfo(p2);
//     posInfo pos_info2 = m_posInfo;
//     bool    alongRoadFlag = getPosInfo::isSameDirectionWithRoad(p1, heading1);
//     bool    alongRoadFlag2 = getPosInfo::isSameDirectionWithRoad(p2, heading2);
//     (*RoadID) = pos_info2.roadId;
//     if (alongRoadFlag) {
//         if (((pos_info2.trackS - pos_info1.trackS) > 0) && (alongRoadFlag2)) {
//             return 1;
//         }
//     } else {
//         if (((pos_info2.trackS - pos_info1.trackS) < 0) && (!alongRoadFlag2)) {
//             return 2;
//         }
//     }
//     return 0;
// }

// TODO isTurnBack:gx 20220127
// need scenario to test
bool getPosInfo::isInSec2_46_6(Point p1,
                               Point p2,
                               float heading1,
                               float heading2,
                               int*  RoadID,float*  Roadheading)
{
    bool    isInRoad = getInertialPosInfo(p1);
    posInfo pos_info1 = m_posInfo;
    getInertialPosInfo(p2);
    posInfo pos_info2 = m_posInfo;
    bool    alongRoadFlag = getPosInfo::isSameDirectionWithRoad(p1, heading1);
    bool    alongRoadFlag2 = getPosInfo::isSameDirectionWithRoad(p2, heading2);
    if (pos_info1.roadId==pos_info2.roadId)
    {
        if (alongRoadFlag) {
            if (((pos_info2.trackS - pos_info1.trackS) > 0) && (alongRoadFlag2)) {
                return 1;
            }
        } else {
            if (((pos_info2.trackS - pos_info1.trackS) < 0) && (!alongRoadFlag2)) {
                return 2;
            }
        }
    }
    else
    {
        
    }
    return 0;
}

// TODO isInRailWithoutTrafficSignal:gx 20220127
// can not test
bool getPosInfo::isInSec1_46_2(Point p)
{
    bool isInRail = getPosInfo::isInSpecialLaneType(
        p, OpenDrive::EnLaneType::ODR_LANE_TYPE_RAIL);
    int TrafficSignalType1 = 1000001;
    int TrafficSignalType2 = 1000002;
    if (isInRail) {
        OpenDrive::Signal* myFirstSignal = m_posInfo.road->getFirstSignal();
        if ((myFirstSignal->mType == TrafficSignalType1) ||
            (myFirstSignal->mType == TrafficSignalType2)) {
            return 0;
        } else {
            while (myFirstSignal->getRight()) {
                OpenDrive::Signal* mynextSignal =
                    (OpenDrive::Signal*)myFirstSignal->getRight();
                myFirstSignal = mynextSignal;
                if ((myFirstSignal->mType == TrafficSignalType1) ||
                    (myFirstSignal->mType == TrafficSignalType2)) {
                    return 0;
                } else if (myFirstSignal->getRight() == NULL) {
                    break;
                }
            }
            return 1;
        }
    }
    return 0;
}

// wh
//  return 1 means increasing dir along road s
//        -1 means decreasing dir of road s
//  int getPosInfo::drivingDirRelToRoad(bool isRHT) {
//    if (m_posInfo.laneId < 0 && isRHT)
//      return 1;
//    if (m_posInfo.laneId > 0 && !isRHT)
//      return 1;
//    return -1;
//  }
//!==============================================================================================================
int getPosInfo::drivingDirRelToRoad(bool isRHT)
{
    if (m_posInfo.laneId < 0 && isRHT)
        return 1;
    if (m_posInfo.laneId > 0 && !isRHT)
        return 1;
    return -1;
}

// wh
std::vector<OpenDrive::Signal*> getPosInfo::getAllSpeedSignsOfRoad(
    OpenDrive::RoadHeader* road)
{
    std::vector<OpenDrive::Signal*> speedSigns;
    const int                       speedSignFlag = 274;
    if (road) {
        OpenDrive::Signal* signal = road->getFirstSignal();
        while (signal) {
            ;
            if (signal->mType == speedSignFlag)
                speedSigns.push_back(signal);
            signal = (OpenDrive::Signal*)signal->getRight();
        }
    }
    sort(speedSigns.begin(),
         speedSigns.end(),
         [](OpenDrive::Signal* a, OpenDrive::Signal* b) {
             return a->mS < b->mS;
         });
    std::cout << "the size of speed sign is: " << speedSigns.size()
              << std::endl;
    return speedSigns;
}

// wh
std::vector<OpenDrive::Signal*> getPosInfo::getLiftSpeedSignsOfRoad(
    OpenDrive::RoadHeader* road)
{
    std::vector<OpenDrive::Signal*> LiftspeedSigns;
    const int                       LiftspeedSignFlag = 278;
    if (road) {
        OpenDrive::Signal* signal = road->getFirstSignal();
        while (signal) {
            if (signal->mType == LiftspeedSignFlag)
                LiftspeedSigns.push_back(signal);
            signal = (OpenDrive::Signal*)signal->getRight();
        }
    }
    sort(LiftspeedSigns.begin(),
         LiftspeedSigns.end(),
         [](OpenDrive::Signal* a, OpenDrive::Signal* b) {
             return a->mS < b->mS;
         });
    std::cout << "the size of lift speed sign is: " << LiftspeedSigns.size()
              << std::endl;
    return LiftspeedSigns;
}

// wh
bool getPosInfo::hasSpeedSignLimit(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    OpenDrive::RoadHeader* road = m_posInfo.road;

    auto speedSigns = getAllSpeedSignsOfRoad(road);
    int  drivingDir = drivingDirRelToRoad();

    double curS = m_posInfo.trackS;

    if (drivingDir == 1) {
        for (int i = speedSigns.size() - 1; i >= 0; --i) {
            auto         sign = speedSigns[i];
            double       signS = sign->mS;
            unsigned int signDir = sign->mDir;
            if (signDir == 0 && curS > signS) {
                return true;
            }
        }
    }

    if (drivingDir == -1) {
        for (int i = 0; i < speedSigns.size(); ++i) {
            auto         sign = speedSigns[i];
            double       signS = sign->mS;
            unsigned int signDir = sign->mDir;
            if (signDir == 1 && curS < signS) {
                return true;
            }
        }
    }
    OpenDrive::RoadHeader* lastRoad = NULL;
    int                    lastRoadDir = -1;
    if (drivingDir == 1) {
        if (road->getPredecessorNode()) {
            lastRoad = road->getPredecessor();
            lastRoadDir = (int)road->getPredecessorDir();
        }
    } else {
        if (road->getSuccessorNode()) {
            lastRoad == road->getSuccessor();
            lastRoadDir = (int)road->getSuccessorDir();
        }
    }

    while (lastRoad) {
        speedSigns = getAllSpeedSignsOfRoad(lastRoad);
        if (lastRoadDir == 0) {
            for (int i = 0; i < speedSigns.size(); ++i) {
                auto         sign = speedSigns[i];
                unsigned int signDir = sign->mDir;
                if (signDir == 1) {
                    return true;
                }
            }
        }

        if (lastRoadDir == 1) {
            for (int i = speedSigns.size() - 1; i >= 0; --i) {
                auto         sign = speedSigns[i];
                unsigned int signDir = sign->mDir;
                if (signDir == 0) {
                    return true;
                }
            }
        }

        if (lastRoadDir == 0) {
            if (road->getSuccessorNode()) {
                auto roadS = lastRoad->getSuccessor();
                lastRoadDir = (int)lastRoad->getSuccessorDir();
                lastRoad = roadS;
            }

        } else if (lastRoadDir == 1) {
            if (road->getPredecessorNode()) {
                auto roadP = lastRoad->getPredecessor();
                lastRoadDir = (int)lastRoad->getPredecessorDir();
                lastRoad = roadP;
            }
        }
    }

    return false;
}

// wh
bool getPosInfo::isInSce2_45_6(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_TOWN) {
        return false;
    }

    if (hasSpeedSignLimit(p)) {
        return false;
    }
    Lane* midLane = m_manager.getLaneSection()->getLaneFromId(0);
    if (midLane->getFirstRoadMark() &&
        midLane->getFirstRoadMark()->getType() != ODR_ROAD_MARK_TYPE_NONE) {
        return false;
    }
    return true;
}

// wh
bool getPosInfo::isInSce2_45_7(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_RURAL) {
        return false;
    }

    if (hasSpeedSignLimit(p)) {
        return false;
    }
    Lane* midLane = m_manager.getLaneSection()->getLaneFromId(0);
    if (midLane->getFirstRoadMark() &&
        midLane->getFirstRoadMark()->getType() != ODR_ROAD_MARK_TYPE_NONE) {
        return false;
    }
    return true;
}

// wh
bool getPosInfo::isInSce2_45_8(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_TOWN) {
        return false;
    }

    if (hasSpeedSignLimit(p)) {
        return false;
    }

    Lane* myLane = m_manager.getLane();
    if (myLane->getType() != ODR_LANE_TYPE_DRIVING) {
        return false;
    }
    int   ID = myLane->getId();
    Lane* myfind = myLane;
    while (myfind->getLeft()) {
        Lane* leftLane = (Lane*)(myfind->getLeft());
        if (leftLane->getId() * ID <= 0)
            break;
        if (leftLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = leftLane;
    }

    myfind = myLane;
    while (myfind->getRight()) {
        Lane* rightLane = (Lane*)(myfind->getRight());
        if (rightLane->getId() * ID <= 0)
            break;
        if (rightLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = rightLane;
    }
    return true;
}

// wh
bool getPosInfo::isInSce2_45_9(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }

    if (m_manager.getRoadType() != ODR_ROAD_TYPE_RURAL) {
        return false;
    }

    if (hasSpeedSignLimit(p)) {
        return false;
    }
    Lane* myLane = m_manager.getLane();
    if (myLane->getType() != ODR_LANE_TYPE_DRIVING) {
        return false;
    }
    int   ID = myLane->getId();
    Lane* myfind = myLane;
    while (myfind->getLeft()) {
        Lane* leftLane = (Lane*)(myfind->getLeft());
        if (leftLane->getId() * ID <= 0)
            break;
        if (leftLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = leftLane;
    }

    myfind = myLane;
    while (myfind->getRight()) {
        Lane* rightLane = (Lane*)(myfind->getRight());
        if (rightLane->getId() * ID <= 0)
            break;
        if (rightLane->getType() == ODR_LANE_TYPE_DRIVING) {
            return false;
        }
        myfind = rightLane;
    }
    return true;
}

// wh
bool getPosInfo::isInSce2_46_3(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    double curv = m_manager.getCurvature();
    if (curv > 0.02 || curv < -0.02) {
        return true;
    }
    return false;
}

// wh
bool getPosInfo::isInSpecialRoadType(Point p, OpenDrive::EnRoadType roadType)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    if (m_manager.getRoadType() == roadType)
        return true;
    return false;
}

// wh
bool getPosInfo::isInSce2_81_x(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    return isInSpecialRoadType(p, ODR_ROAD_TYPE_MOTORWAY);
}

// wh
bool getPosInfo::isInSce1_42_1(Point p)
{
    return hasSpeedSignLimit(p);
}

// wh
bool getPosInfo::isInSce2_45_5(Point p)
{
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad) {
        return false;
    }
    OpenDrive::RoadHeader* road = m_posInfo.road;

    auto   speedSigns = getAllSpeedSignsOfRoad(road);
    auto   liftSpeedSigns = getLiftSpeedSignsOfRoad(road);
    int    drivingDir = drivingDirRelToRoad();
    double curS = m_posInfo.trackS;

    if (drivingDir == 1) {
        double tempSpeedSignS = 0.;
        double tempLiftSpeedSignS = 0.;
        for (int i = speedSigns.size() - 1; i >= 0; --i) {
            auto         sign = speedSigns[i];
            double       signS = sign->mS;
            unsigned int signDir = sign->mDir;
            if (signDir == 0 && curS > signS) {
                tempSpeedSignS = signS;
            }
        }
        for (int i = liftSpeedSigns.size() - 1; i >= 0; --i) {
            auto         liftSign = liftSpeedSigns[i];
            double       liftSignS = liftSign->mS;
            unsigned int signDir = liftSign->mDir;
            if (signDir == 0 && curS > liftSignS) {
                tempLiftSpeedSignS = liftSignS;
            }
        }
        cout << tempSpeedSignS << endl;
        cout << tempLiftSpeedSignS << endl;
        if (tempLiftSpeedSignS > tempSpeedSignS) {
            cout << "1" << endl;
            return true;
        }
    }

    if (drivingDir == -1) {
        double tempSpeedSignS = 0.;
        double tempLiftSpeedSignS = 0.;
        for (int i = 0; i < speedSigns.size(); ++i) {
            auto         sign = speedSigns[i];
            double       signS = sign->mS;
            unsigned int signDir = sign->mDir;
            if (signDir == 1 && curS < signS) {
                tempSpeedSignS = signS;
            }
        }
        for (int i = 0; i < liftSpeedSigns.size(); ++i) {
            auto         liftSign = liftSpeedSigns[i];
            double       liftSignS = liftSign->mS;
            unsigned int signDir = liftSign->mDir;
            if (signDir == 1 && curS < liftSignS) {
                tempLiftSpeedSignS = liftSignS;
            }
        }
        if (tempLiftSpeedSignS < tempSpeedSignS) {
            return true;
        }
    }

    OpenDrive::RoadHeader* lastRoad = NULL;
    int                    lastRoadDir = -1;
    if (drivingDir == 1) {
        if (road->getPredecessorNode()) {
            lastRoad = road->getPredecessor();
            lastRoadDir = (int)road->getPredecessorDir();
        }
    } else {
        if (road->getSuccessorNode()) {
            lastRoad == road->getSuccessor();
            lastRoadDir = (int)road->getSuccessorDir();
        }
    }

    while (lastRoad) {
        speedSigns = getAllSpeedSignsOfRoad(lastRoad);
        liftSpeedSigns = getLiftSpeedSignsOfRoad(road);
        if (lastRoadDir == 0) {
            double tempSpeedSignS = 0.;
            double tempLiftSpeedSignS = 0.;
            for (int i = 0; i < speedSigns.size(); ++i) {
                auto         sign = speedSigns[i];
                unsigned int signDir = sign->mDir;
                if (signDir == 1) {

                    tempSpeedSignS = sign->mS;
                }
            }
            for (int i = 0; i < liftSpeedSigns.size(); ++i) {
                auto         liftSign = speedSigns[i];
                unsigned int signDir = liftSign->mDir;
                if (signDir == 1) {

                    tempLiftSpeedSignS = liftSign->mS;
                }
            }
            if (tempLiftSpeedSignS < tempSpeedSignS) {
                return true;
            }
        }

        if (lastRoadDir == 1) {
            double tempSpeedSignS = 0.;
            double tempLiftSpeedSignS = 0.;
            for (int i = speedSigns.size() - 1; i >= 0; --i) {
                auto         sign = speedSigns[i];
                unsigned int signDir = sign->mDir;
                if (signDir == 0) {
                    tempSpeedSignS = sign->mS;
                }
            }
            for (int i = liftSpeedSigns.size() - 1; i >= 0; --i) {
                auto         liftSign = speedSigns[i];
                unsigned int signDir = liftSign->mDir;
                if (signDir == 0) {
                    tempLiftSpeedSignS = liftSign->mS;
                }
            }
            if (tempLiftSpeedSignS > tempSpeedSignS) {
                return true;
            }
        }

        if (lastRoadDir == 0) {
            if (road->getSuccessorNode()) {
                auto roadS = lastRoad->getSuccessor();
                lastRoadDir = (int)lastRoad->getSuccessorDir();
                lastRoad = roadS;
            }

        } else if (lastRoadDir == 1) {
            if (road->getPredecessorNode()) {
                auto roadP = lastRoad->getPredecessor();
                lastRoadDir = (int)lastRoad->getPredecessorDir();
                lastRoad = roadP;
            }
        }
    }
    return false;
}

//xyz
bool getPosInfo::isInSec2_46_1(Point p){
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad){
        return false;
    }else
    {
        Lane* tempLane = m_posInfo.lane; 
        if (tempLane->getType()==ODR_LANE_TYPE_BIKING){  
            return true;
        }else{
            return false;
        }
    }
}

//xyz
bool getPosInfo::isInSec1_51_1(Point p,float vehicleSpeed,float heading){
    bool function3_1(int tempLaneID,RoadHeader* header,double ts,float heading,double roadHeading);
    bool function3_2(int tempLaneID,RoadHeader* header,double ts,float heading,double roadHeading);
    bool isInRoad = getInertialPosInfo(p);
    if (!isInRoad){
        return false;
    }

    if (vehicleSpeed!=0){
        return false;
    }
   
    RoadHeader* header = m_posInfo.road;
    int junctionNum = header->getJunctionNo(); 
    // cout << "junction num: " << junctionNum << endl;
    if(junctionNum!=-1){
        return false;
    }
    // cout<< "road id :" <<header->getId() << endl;
    LaneSection* ls = m_posInfo.section;
    // cout<< "lane section s :" << ls->mS << endl;
    Lane* tempLane = m_posInfo.lane;
    int tempLaneID = tempLane->getId();
    // cout<< "lane id :" << tempLane->getId() << endl;

    RoadLink* predecessorLink = header->getPredecessorLink(ODR_TYPE_ROAD);
    if (predecessorLink==NULL){
        predecessorLink = header->getPredecessorLink(ODR_TYPE_JUNCTION);
    }
    RoadLink* successorLink = header->getSuccessorLink(ODR_TYPE_ROAD);
    if (successorLink==NULL){
        successorLink = header->getSuccessorLink(ODR_TYPE_JUNCTION);
    }
    
    double ts = m_posInfo.trackS;
    double tt = m_posInfo.trackT;
    if(successorLink!=NULL && predecessorLink==NULL){
        unsigned short successorType = successorLink->getElemType();  
        if(successorType==ODR_TYPE_JUNCTION){
            double roadHeading = m_posInfo.roadheading;
            bool tempFlag = function3_1(tempLaneID,header,ts,heading,roadHeading); 
            return tempFlag;
        }else{
            return false;
        }
    }
    if(successorLink==NULL && predecessorLink!=NULL){
        unsigned short predecessorType = predecessorLink->getElemType();  
        if (predecessorType==ODR_TYPE_JUNCTION){  
            double roadHeading = m_posInfo.roadheading;
            bool tempFlag = function3_2(tempLaneID,header,ts,heading,roadHeading); 
            return tempFlag;
        }else{
            return false;
        }
    }
    if(successorLink==NULL && predecessorLink==NULL){
        return false;
    }
    if(successorLink!=NULL && predecessorLink!=NULL){
        unsigned short successorType = successorLink->getElemType();  
        unsigned short predecessorType = predecessorLink->getElemType();  
        // cout<<"successorType: "<< successorType << endl;
        // cout<<"predecessorType: "<< predecessorType << endl;
        if(successorType==ODR_TYPE_JUNCTION && predecessorType!=ODR_TYPE_JUNCTION){  
            double roadHeading = m_posInfo.roadheading;
            bool tempFlag = function3_1(tempLaneID,header,ts,heading,roadHeading); 
            return tempFlag;
        }else if (successorType!=ODR_TYPE_JUNCTION && predecessorType==ODR_TYPE_JUNCTION)
        {
            double roadHeading = m_posInfo.roadheading;
            bool tempFlag = function3_2(tempLaneID,header,ts,heading,roadHeading); 
            return tempFlag;
        }else if (successorType==ODR_TYPE_JUNCTION && predecessorType==ODR_TYPE_JUNCTION)
        {
            return true;
        }else{
            return false;
        }
    } 

}

//xyz
bool getPosInfo::isInSec3_6_6(Point p,float vehicleSpeed,int vehicleType,Point p2,float heading1){  
    bool isInRoad = getInertialPosInfo(p);  
    posInfo pos_info1 = m_posInfo;
    if (!isInRoad){
        return false;
    }
    
    isInRoad = getInertialPosInfo(p2);  
    posInfo pos_info2 = m_posInfo;
    if (!isInRoad){
        return false;
    }

    RoadHeader* header = pos_info1.road; 
    LaneSection* ls = pos_info1.section;
    // cout << "road id : " << header->getId() << endl;
    // cout << "lane section : "<< ls->mS << endl;
    // cout << "lane id : " << m_posInfo.laneId << endl;

    unsigned int laneType = pos_info1.laneType;
    RoadType* rt = header->getFirstRoadType();
    unsigned int roadType = rt->mType;
    // cout << "road type:" << roadType << endl;
    // cout << "start pos: " << rt->mStartPos << endl;
    if (roadType==ODR_ROAD_TYPE_MOTORWAY){  
        return false;
    }

    RoadType* nextRT = reinterpret_cast<RoadType*>(rt->getRight());
    while (nextRT!=NULL){
        unsigned int roadType = nextRT->mType;
        // cout << "next road type:" << roadType << endl;
        if (roadType==ODR_ROAD_TYPE_MOTORWAY){ 
            return false;
        }
        nextRT = reinterpret_cast<RoadType*>(nextRT->getRight());
    }
        
    if(vehicleSpeed!=0){
        return false;
    }
    if(vehicleType!=14){
        return false;
    }
    int roadId1 = pos_info1.roadId;
    int roadId2 = pos_info2.roadId;
    int laneId1 = pos_info1.laneId;
    int laneId2 = pos_info2.laneId;
    if (roadId1==roadId2 && laneId1*laneId2>0){
        double roadHeading = pos_info1.roadheading;
        float pi = atan(1) * 4;
        float CarHeading = roadHeading - heading1;
        cout << "carHeading: " << CarHeading << endl;
        if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0))
        {
            double ts1 = pos_info1.trackS;  
            double ts2 = pos_info2.trackS;  
            if (laneId1>0){  
                if (ts2>=ts1){
                    return true;
                }else{
                    return false;
                }
            }else if (laneId1<0)  
            {
                if (ts2<=ts1){
                    return true;
                }else{
                    return false;
                }
            }else{
                return false;
            } 
        }else{
            double ts1 = pos_info1.trackS; 
            double ts2 = pos_info2.trackS; 
            if (laneId1>0){  
                if (ts2<=ts1){
                    return true;
                }else{
                    return false;
                }
            }else if (laneId1<0)  
            {
                if (ts2>=ts1){
                    return true;
                }else{
                    return false;
                }
            }else{
                return false;
            }
        }    
    }else{  
        return false;
    }
}

//xyz
int getPosInfo::isInSec2_47_1(Point pa11,Point pa12,float heading1){
    bool isInRoad = getInertialPosInfo(pa11);  
    posInfo pos_info11 = m_posInfo;
    if (!isInRoad){
        return -1;
    }
    isInRoad = getInertialPosInfo(pa12);  
    posInfo pos_info12 = m_posInfo;
    if (!isInRoad){
        return -1;
    }
    
    int pa11_LaneId = pos_info11.laneId;
    int pa12_LaneId = pos_info12.laneId;

    int pa11_RoadId = pos_info11.roadId;
    int pa12_RoadId = pos_info12.roadId;
    
    Lane* lane11 = pos_info11.lane;
    Lane* lane11_p = lane11->getPredecessor();
    Lane* lane11_s = lane11->getSuccessor();
    int lane11_p_id;
    int lane11_s_id;
    if (lane11_p!=NULL)
    {
        lane11_p_id = lane11_p->getId();  
    }else
    {
        lane11_p_id = 99999;
    }
    if (lane11_s!=NULL)
    {
        lane11_s_id = lane11_s->getId();  
    }else
    {
        lane11_s_id = 99999;
    }
    
    double roadHeading = pos_info11.roadheading;
    float pi = atan(1) * 4;
    float CarHeading = roadHeading - heading1;
    if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0))
    {   
        if (lane11_s_id!=99999)
        {
            pa11_LaneId = lane11_s_id;
        }else
        {
            return -1;
        }
        
    }else{
        if (lane11_p_id!=99999)
        {
            pa11_LaneId = lane11_p_id;
        }else
        {
            return -1;
        }   
    }
    
    int tempNum = isInSec2_57_5(pa11_LaneId,pa12_LaneId,heading1,roadHeading);
    return tempNum;
}

//xyz
int getPosInfo::isInSec2_57_5(int laneId1,int laneId2,float heading1,double roadHeading){
    float pi = atan(1) * 4;
    float CarHeading = roadHeading - heading1;
    if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0)) {  
        if (laneId1>0)
        {
            if (laneId1>laneId2)
            {
                return 1;
            }else if (laneId1< laneId2)
            {
                return 2;
            }else{
                return 0;
            }
        }
        if (laneId1<0)
        {
            if (laneId1<laneId2)
            {
                return 1;
            }else if (laneId1>laneId2)
            {
                return 2;
            }else{
                return 0;
            }            
        }
        
    }else{  
        if (laneId1>0)
        {
            if (laneId1< laneId2)
            {
                return 1;
            }else if (laneId1> laneId2)
            {
                return 2;
            }else{
                return 0;
            } 
        }
        if (laneId1<0)
        {
            if (laneId1>laneId2)
            {
                return 1;
            }else if (laneId1<laneId2)
            {
                return 2;
            }else{
                return 0;
            }
        }

    }
}

// ..................................................................................
//xyz
bool function3_1(int tempLaneID,RoadHeader* header,double ts,float heading,double roadHeading){
    if (tempLaneID>=0){
        return false;
    }
    Signal* firstSignal = header->getFirstSignal();
    if(firstSignal==NULL){
        return true;
    }
    int signalType = firstSignal->mType;
    Signal* nextsignal = firstSignal;
    double signalS;
    double signalHeight;
    unsigned int flag = 0;
    if (signalType!=294){  
        while (signalType!=294){
            nextsignal = reinterpret_cast<Signal*>(nextsignal->getRight());
            if (nextsignal==NULL){
                return true;
            }
            signalType = nextsignal->mType;
            if (signalType==294){
                signalS = nextsignal->mS;
                signalHeight = nextsignal->mHeight;
                flag = 1;
                break;
            }
        }
        if (flag==0){
            return true;
        }
    }else{
        signalS = firstSignal->mS; 
        signalHeight = firstSignal->mHeight;
    }
    
    float pi = atan(1) * 4;
    float CarHeading = roadHeading - heading;
    cout << "carHeading: " << CarHeading <<"** roadHeading: "<<roadHeading<< endl;
    if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0))
    {
        double tempNum = signalS-signalHeight/2;
        if (ts<tempNum){
            return true;
        }else{
            return false;
        }
    }else{
        double tempNum = signalS+signalHeight/2;
        if (ts>tempNum){
            return true;
        }else{
            return false;
        }
    }
}

//xyz
bool function3_2(int tempLaneID,RoadHeader* header,double ts,float heading,double roadHeading){
    if (tempLaneID<=0){
        // cout << "lane id > 0" << endl;
        return false;
    }
    Signal* firstSignal = header->getFirstSignal();
    if(firstSignal==NULL){
        return true;
    }
    int signalType = firstSignal->mType;
    Signal* nextsignal = firstSignal;
    double signalS;
    double signalHeight;
    unsigned int flag = 0;
    if (signalType!=294){  
        while (signalType!=294){
            nextsignal = reinterpret_cast<Signal*>(nextsignal->getRight());
            if (nextsignal==NULL){
                return true;
            }
            signalType = nextsignal->mType;
            if (signalType==294){
                signalS = nextsignal->mS;
                signalHeight = nextsignal->mHeight;
                flag = 1;
                break;
            }
        }
        if (flag==0){
            return true;
        }
    }else{
        signalS = firstSignal->mS;  // 第一个signal的s值
        signalHeight = firstSignal->mHeight;
    }
    float pi = atan(1) * 4;
    float CarHeading = roadHeading - heading;
    cout << "roadHeading: " <<roadHeading << endl;
    cout << "heading: " << heading << endl;
    cout << "carHeading: " << CarHeading << endl;
    if (((pi / 2) > CarHeading > 0) || ((-pi / 2) < CarHeading < 0))
    {
        double tempNum = signalS+signalHeight/2;
        cout << "ts: " <<ts<<"tempNum: "<< tempNum<<endl;
        if (ts>tempNum){
            return true;
        }else{
            return false;
        }
    }else{
        double tempNum = signalS-signalHeight/2;
        cout << "ts: " <<ts<<"tempNum: "<< tempNum<<endl;
        if (ts<tempNum){
            return true;
        }else{
            return false;
        }
    }     
}

// if there is a front car and DUT's speed is higher than 100 km/h:gx20220225
bool getPosInfo::function2_80_1(Point  p1,
                                Point  p2,
                                float  heading1,
                                float  heading2,
                                float* length,
                                float  speed)
{
    if ((speed - 100) > 0) {
        bool isFrontCarFlag =
            getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
        if (isFrontCarFlag) {
            return 1;
        }
    }
    return 0;
}

// if there is a front car and DUT's speed is higher than 100 km/h:gx 20220225 
bool getPosInfo::function2_80_2(Point  p1,
                                Point  p2,
                                float  heading1,
                                float  heading2,
                                float* length,
                                float  speed)
{
    if ((speed - 100) <= 0)
        {
            bool isFrontCarFlag =
                getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
            if (isFrontCarFlag) {
                return 1;
            }
        }
    return 0;
}

// TODO if there is a front car at town road:gx 20220225
//the scenario has no road type so that the result is false
bool getPosInfo::function3_7_1(Point  p1,
                               Point  p2,
                               float  heading1,
                               float  heading2,
                               float* length)
{
    bool isTownRoadFlag = getPosInfo::isInSpecialRoadType(
        p1, OpenDrive::EnRoadType::ODR_ROAD_TYPE_TOWN);
    if (isTownRoadFlag) {
        bool isFrontCarFlag =
            getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
        if (isFrontCarFlag) {
            return 1;
        }
    }
    return 0;
}

// TODO if there is a front car at night:gx 20220225
bool getPosInfo::function3_7_4(Point  p1,
                               Point  p2,
                               float  heading1,
                               float  heading2,
                               float* length,
                               float  time)
{
    int Night=1;
    //TODO 
    if (time == Night) {
        bool isFrontCarFlag =
            getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
        if (isFrontCarFlag) {
            return 1;
        }
    }
    return 0;
}

// TODO if there is a front car at bad weather:gx 20220309
bool getPosInfo::function3_7_5(Point  p1,
                               Point  p2,
                               float  heading1,
                               float  heading2,
                               float* length,
                               float  visibility,
                               int snow,
                               int rainy,
                               int hail)
{
    if ((visibility <50)||(snow==1)||(rainy==1)||(hail==1)) {
        bool isFrontCarFlag =
            getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
        if (isFrontCarFlag) {
            return 1;
        }
    }
    return 0;
}

// TODO if there is a front car and conput the distance:wh 20220309
bool getPosInfo::function1_43_1(Point  p1,
                               Point  p2,
                               float  heading1,
                               float  heading2,
                               float* length)
{
    bool isFrontCarFlag =
        getPosInfo::isFrontCar(p1, p2, heading1, heading2, length);
    if (isFrontCarFlag) {
        return 1;
    }
    return 0;
}

// isInJunctionWithoutTrafficSignal:gx 20220301
bool getPosInfo::isInSec1_47_2(Point p)
{
    getInertialPosInfo(p);
    bool isInJunction = 0;
    if (m_posInfo.junctionId != -1) {
        isInJunction = 1;
    }
    int TrafficSignalType1 = 1000001;
    int TrafficSignalType2 = 1000002;
    if (isInJunction&&(m_posInfo.road->getFirstSignal())) {
        OpenDrive::Signal* myFirstSignal = m_posInfo.road->getFirstSignal();
        std::cout << "3: " << std::endl;
        std::cout << "3: " << myFirstSignal->mId<<std::endl;
        if ((myFirstSignal->mType == TrafficSignalType1) ||
            (myFirstSignal->mType == TrafficSignalType2)) {
                std::cout << "4: " << std::endl;
            return 0;
        } else {
            std::cout << "5: " << std::endl;
            while (myFirstSignal->getRight()) {
                std::cout << "6: " << std::endl;
                OpenDrive::Signal* mynextSignal =
                    (OpenDrive::Signal*)myFirstSignal->getRight();
                myFirstSignal = mynextSignal;
                if ((myFirstSignal->mType == TrafficSignalType1) ||
                    (myFirstSignal->mType == TrafficSignalType2)) {
                    return 0;
                } else if (myFirstSignal->getRight() == NULL) {
                    break;
                }
            }
            return 1;
        }
    }
    else if (m_posInfo.road->getFirstSignal()==NULL)
    {
        return 1;
    }
    return 0;
}

// jm
posInfo getPosInfo::getInfo() const
{
    return m_posInfo;
}
