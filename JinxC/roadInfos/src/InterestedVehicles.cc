/* ===================================================
 *  file:       InterestedVehicles.cc
 * ---------------------------------------------------
 *  purpose:    find 10 interested vehicles
 * ---------------------------------------------------
 *  author :    MengXia Hu & Yue Zheng @ CICV
 * ===================================================
 */


/* ====== INCLUSIONS ====== */
#include "InterestedVehicles.hh"

namespace Module
{

double
InterestedVehicles::calRoadSDiff(int drvDir, double ownS, double start, double end)
{
    if (drvDir < 0)
    {
        return end - ownS;
    }
    else
    {
        return ownS - start;
    }
}

bool 
InterestedVehicles::findFrontVehicle(int offLane)
{
    for (std::list<KeyPoint>::iterator i = route_list.begin(); i != route_list.end(); ++i)
    {
        for (std::map<int, objVehicle>::const_iterator mapIt = vehicle_map.begin(); mapIt != vehicle_map.end(); ++mapIt )
        {
            if (mapIt->first == ownCurState.ownObjState->base.id)
            {
                continue;
            }
            // if vehicle on this road
            if (mapIt->second.curRoad && mapIt->second.curRoad->mId == i->curRoad->mId && i->curLane)
            {
                int interestedLaneId = i->curLane->mId + i->drvDir * offLane;
                // if vehicle on this lane
                if ( i->curLaneSec->getLaneFromId(interestedLaneId) && interestedLaneId == mapIt->second.curLane->mId )
                {
                    // vehicle and road has similar angle
                    if (alignToRoad(mapIt->second.curHeadingDiff))
                    {
                        double laneSecStart =  i->curLaneSec->mS;
                        double laneSecEnd = i->curLaneSec->mSEnd;
                        // if it is the first keypoint, check whether it is ahead of Ego. if it's not, no further check needed.
                        if ( mapIt->second.curLaneSec->mS == i->curLaneSec->mS && 
                           ( (i == route_list.begin() && (-i->drvDir)*ownCurState.ownRoadS < (-i->drvDir)*mapIt->second.curS ) || i != route_list.begin()) )
                        {
                            double tmpDis = i->distance - calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);

                            // if already exist a candidate, choose the closer one
                            if (vehPackage.Exist == true && tmpDis > vehPackage.disToOwn)
                            {
                                continue;
                            }

                            //vehPackage.disToOwn = i->distance - calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);
                            //std::cerr << "laneSecStart: "<< laneSecStart << std::endl;
                            //std::cerr << "laneSecEnd: "<< laneSecEnd << std::endl;
                            //std::cerr << "i->distance: "<< i->distance << std::endl;
                            //std::cerr << "calRoadSDiff: "<< calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd) << std::endl;
                            vehPackage.Exist = true;
                            vehPackage.playerId = mapIt->second.objState->base.id;
                            vehPackage.speed = getSpeed(mapIt->second.objState);
                            vehPackage.accel = getAccel(mapIt->second.objState);
                            vehPackage.roadS = mapIt->second.curS;
                            vehPackage.roadT = mapIt->second.curT;
                            vehPackage.disToOwn = tmpDis;
                            vehPackage.type = mapIt->second.objState->base.type;
                            vehPackage.roadId = mapIt->second.curRoad->mId;
                            vehPackage.laneId = mapIt->second.curLane->mId;
                            vehPackage.laneWidth = mapIt->second.curLaneWidth;
                            vehPackage.laneOffset = mapIt->second.curLaneOffset;
                            vehPackage.laneCurve = mapIt->second.curLaneCurve;
                            vehPackage.headingDiff = mapIt->second.curHeadingDiff;  //WANING : HMX ADDED!
                            //vehPackage.roadPos;
                            vehPackage.geo = &(mapIt->second.objState->base.geo);
                            vehPackage.pos = &(mapIt->second.objState->base.pos);
                            vehPackage.objState = mapIt->second.objState;

                        }
                    }
                }

            }
        }
        if (vehPackage.Exist == true)
        {
            return true;
        }
    }
    return false;
}

bool 
InterestedVehicles::findRearVehicle(int offLane)
{
    std::list<KeyPoint> route_list_tmp = route_list_past;
    route_list_tmp.push_front(route_list.front());

    double runningS = 0;
    double startSOffset = 0;

    for (std::list<KeyPoint>::iterator i = route_list_tmp.begin(); i != route_list_tmp.end(); ++i)
    {
        // add laneSec length to runningS
        runningS += i->curLaneSec->mSEnd - i->curLaneSec->mS;

        if (i == route_list_tmp.begin())
        {
            startSOffset = i->distance;
        }
        for (std::map<int, objVehicle>::const_iterator mapIt = vehicle_map.begin(); mapIt != vehicle_map.end(); ++mapIt )
        {
            if (mapIt->first == ownCurState.ownObjState->base.id)
            {
                continue;
            }
            // if vehicle on this road
            if (mapIt->second.curRoad && mapIt->second.curRoad->mId == i->curRoad->mId && i->curLane)
            {
                int interestedLaneId = i->curLane->mId + i->drvDir * offLane;
                // if vehicle on this lane
                if ( i->curLaneSec->getLaneFromId(interestedLaneId) && interestedLaneId == mapIt->second.curLane->mId )
                {
                    if (alignToRoad(mapIt->second.curHeadingDiff))
                    {
                        double laneSecStart =  i->curLaneSec->mS;
                        double laneSecEnd = i->curLaneSec->mSEnd;
                        // if it is the first keypoint, check whether it is behind Ego. if it's not, no further check needed.
                        if ( mapIt->second.curLaneSec->mS == i->curLaneSec->mS && 
                           ( (i == route_list.begin() && (-i->drvDir)*ownCurState.ownRoadS > (-i->drvDir)*mapIt->second.curS ) || i != route_list.begin()) )
                        {
                            double tmpDis = runningS - startSOffset - calRoadSDiff(-i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);

                            // if already exist a candidate, choose the closer one
                            if ( tmpDis <= 0 || (vehPackage.Exist == true && tmpDis > vehPackage.disToOwn) )
                            {
                                continue;
                            }
                            //vehPackage.disToOwn = runningS - startSOffset - calRoadSDiff(-i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);
                            //std::cerr << "runningS: "<< runningS << std::endl;
                            //std::cerr << "startSOffset: "<< startSOffset << std::endl;
                            //std::cerr << "calRoadSDiff: "<< calRoadSDiff(-i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd) << std::endl;
                            vehPackage.Exist = true;
                            vehPackage.playerId = mapIt->second.objState->base.id;
                            vehPackage.speed = getSpeed(mapIt->second.objState);
                            vehPackage.accel = getAccel(mapIt->second.objState);
                            vehPackage.roadS = mapIt->second.curS;
                            vehPackage.roadT = mapIt->second.curT;
                            vehPackage.disToOwn = tmpDis;
                            vehPackage.type = mapIt->second.objState->base.type;
                            vehPackage.roadId = mapIt->second.curRoad->mId;
                            vehPackage.laneId = mapIt->second.curLane->mId;
                            vehPackage.laneWidth = mapIt->second.curLaneWidth;
                            vehPackage.laneOffset = mapIt->second.curLaneOffset;
                            vehPackage.laneCurve = mapIt->second.curLaneCurve;
                            vehPackage.headingDiff = mapIt->second.curHeadingDiff;  //WANING : HMX ADDED!
                            //vehPackage.roadPos;
                            vehPackage.geo = &(mapIt->second.objState->base.geo);
                            vehPackage.pos = &(mapIt->second.objState->base.pos);
                            vehPackage.objState = mapIt->second.objState;
                        }
                    }
                }

            }
        }
        if (vehPackage.Exist == true)
        {
            return true;
        }
    }
    return false;
}

bool 
InterestedVehicles::isCuttingIn(RDB_OBJECT_STATE_t* objState, double width, double dis, double angle)
{
    double speedLat = getSpeed(objState) * sin(angle);
    // std::cerr << "id: "<< objState->base.id << std::endl;
    // std::cerr << "angle: "<< angle << std::endl;
    // std::cerr << "speedLat: "<< speedLat << std::endl;
    // std::cerr << "dis: "<< dis << std::endl;
    // std::cerr << "width: "<< width << std::endl;
    // if (objState->base.id == 2)
    // {
    //     std::cerr << "true of false: "<< (fabs(speedLat) * 2 >= width/2.0 - dis) << std::endl;    
    // }
    // if (objState->base.id == 3)
    // {
    //     std::cerr << "true of false: "<< (fabs(speedLat) * 2 >= width/2.0 + dis) << std::endl;    
    // }
    return speedLat > 0? fabs(speedLat) * 2.5 >= width/2.0 + dis : fabs(speedLat) * 2 >= width/2.0 - dis;
}

bool 
InterestedVehicles::findCutInVehicle(int offLane)
{
    std::set<int> tmp_checked;
    for (std::list<KeyPoint>::iterator i = route_list.begin(); i != route_list.end(); ++i)
    {
        for (std::map<int, objVehicle>::const_iterator mapIt = vehicle_map.begin(); mapIt != vehicle_map.end(); ++mapIt )
        {
            // if checked, no need to check again
            if (tmp_checked.find(mapIt->first) != tmp_checked.end())
            {
                continue;
            }
            tmp_checked.insert(mapIt->first);

            int updated = 0;
            if (mapIt->first != ownCurState.ownObjState->base.id)
            {//std::cerr << "333" << std::endl;
                // if vehicle on this road and within this laneSec
                if (mapIt->second.curRoad 
                    && mapIt->second.curRoad->mId == i->curRoad->mId && i->curLane
                    && mapIt->second.curS >= i->curLaneSec->mS && mapIt->second.curS <= i->curLaneSec->mSEnd)
                {//std::cerr << "444" << std::endl;
                    int interestedLaneId = i->curLane->mId + i->drvDir * offLane;
                    // if vehicle on this lane
                    if ( i->curLaneSec->getLaneFromId(interestedLaneId) && interestedLaneId == mapIt->second.curLane->mId )
                    {//std::cerr << "555" << std::endl;
                        // vehicle and road has similar angle && is going to cutin
                        if (alignToRoad(mapIt->second.curHeadingDiff) && isCuttingIn(mapIt->second.objState, mapIt->second.curLaneWidth, mapIt->second.curLaneOffset, mapIt->second.curHeadingDiff))
                        {
                            double laneSecStart =  i->curLaneSec->mS;
                            double laneSecEnd = i->curLaneSec->mSEnd;
                            // if it is the first keypoint, check whether it is ahead of Ego. if it's not, no further check needed.
                            if ( mapIt->second.curLaneSec->mS == i->curLaneSec->mS && 
                               ( (i == route_list.begin() && (-i->drvDir)*ownCurState.ownRoadS < (-i->drvDir)*mapIt->second.curS ) || i != route_list.begin()) )
                            {
                                double tmpDis = i->distance - calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);

                                // if not further than lv, save it
                                if (!(vehPackage.Exist == true && tmpDis > vehPackage.disToOwn))
                                {
                                    //vehPackage.disToOwn = i->distance - calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd);
                                    //std::cerr << "laneSecStart: "<< laneSecStart << std::endl;
                                    //std::cerr << "laneSecEnd: "<< laneSecEnd << std::endl;
                                    //std::cerr << "i->distance: "<< i->distance << std::endl;
                                    //std::cerr << "calRoadSDiff: "<< calRoadSDiff(i->drvDir, mapIt->second.curS, laneSecStart, laneSecEnd) << std::endl;
                                    InterestedVehicle civ;

                                    civ.Exist = true;
                                    civ.playerId = mapIt->second.objState->base.id;
                                    civ.speed = getSpeed(mapIt->second.objState);
                                    civ.accel = getAccel(mapIt->second.objState);
                                    civ.roadS = mapIt->second.curS;
                                    civ.roadT = mapIt->second.curT;
                                    civ.disToOwn = tmpDis;
                                    civ.type = mapIt->second.objState->base.type;
                                    civ.roadId = mapIt->second.curRoad->mId;
                                    civ.laneId = mapIt->second.curLane->mId;
                                    civ.laneWidth = mapIt->second.curLaneWidth;
                                    civ.laneOffset = mapIt->second.curLaneOffset;
                                    civ.laneCurve = mapIt->second.curLaneCurve;
                                    civ.headingDiff = mapIt->second.curHeadingDiff;  //WANING : HMX ADDED!
                                    civ.geo = &(mapIt->second.objState->base.geo);
                                    civ.pos = &(mapIt->second.objState->base.pos);
                                    civ.objState = mapIt->second.objState;

                                    //WARNING: HMX CODING
                                    civ.findType = 1;
                                    civ.findLaneOffset = offLane;


                                    // update into civ_map
                                    std::cerr << "0709Push "<< civ.playerId << " INTO" << std::endl;
                                    civ_map[civ.playerId] = civ;
                                    updated = 1;
                                }

                            }
                        }
                    }
                    else
                    {
                        // Don't erase other lanes candidates
                        continue;
                    }
                }
            }

            // if not updated, it is no longer a cutin vehicle
            if (!updated)
            {
                std::map<int, InterestedVehicle>::iterator civMapIt = civ_map.find(mapIt->second.objState->base.id);
                if ( civMapIt != civ_map.end() )
                {
                    civ_map.erase(civMapIt);
                    //civMapIt->second.Exist = false;
                }
            }
        }
    }
    std::cerr << "civ_size: "<< civ_map.size() << std::endl;

    return true;
}

bool 
InterestedVehicles::findInterestedVehicle(int type, int offLane)
{
    switch(type)
    {
        case 1:
        return findFrontVehicle(offLane);

        case 2:
        return findRearVehicle(offLane);

        case 3:
        return findCutInVehicle(offLane);

    }

    return false;
}

} // namespace Module