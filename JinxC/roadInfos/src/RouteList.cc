/* ===================================================
 *  file:       RouteList.cc
 * ---------------------------------------------------
 *  purpose:	find and store nearby routelist of HDM
 * ---------------------------------------------------
 *  author :	MengXia Hu & Yue Zheng @ CICV
 * ===================================================
 */


/* ====== INCLUSIONS ====== */
#include "RouteList.hh"

namespace Module
{

void 
RouteList::updateRouteList()
{
    double dist = 0;
    int cnt = 0;

    OpenDrive::RoadHeader* roadNow = 0;
    OpenDrive::LaneSection* laneSecNow = 0;
    OpenDrive::Lane* laneNow = 0;

    roadNow = ownCurState.ownRoadHeader;
    laneSecNow = ownCurState.ownLaneSection;
    if (laneSecNow)
    {
        laneNow = laneSecNow->getLaneFromId(ownCurState.ownLaneId);
    }

    if ( !roadNow || !laneSecNow || !laneNow )
    {
    	std::cerr << "launch safe mode!" << std::endl;
    	route_list.clear();
    	return;
    }

    OpenDrive::RoadHeader* roadNext = 0;
    OpenDrive::LaneSection* laneSecNext = 0;
    OpenDrive::Lane* laneNext = 0; 

    // update pastState
    if (route_list.size())
    {
        ownPastState.ownRoadHeader = route_list.front().curRoad;
        ownPastState.ownLaneSection = route_list.front().curLaneSec;
        ownPastState.ownLane = route_list.front().curLane;
    }

    // pop passed ones
    while ( route_list.size() && 
    		( ownCurState.ownRoadId != route_list.front().curRoad->mId || 
    			( laneNow->mId < 0 && ownCurState.ownRoadS > route_list.front().roadS) || 
    			( laneNow->mId > 0 && ownCurState.ownRoadS < route_list.front().roadS) 
    		) 
    	  ) 
    {
        //std::cerr << "0629ownCurState.ownRoadId "<< ownCurState.ownRoadHeader->mId << std::endl;
        //std::cerr << "0629nextRoadId "<< route_list.front().curRoad->mId << std::endl;
        //std::cerr << "0629laneNow->mId "<< laneNow->mId << std::endl;


        route_list_past.push_front(route_list.front());
        if (route_list_past.size() > 3)
        {
            route_list_past.pop_back();
        }

        route_list.pop_front();
    }
    
    std::list<KeyPoint>::iterator i = route_list.begin(); 

    while ( roadNow->mId && (dist < 200 || cnt < 3) )
    {   
        ++cnt;

        if ( dist == 0 )
        {
            if ( laneNow->mId < 0 )
            {
                dist = roadNow->mLength - ownCurState.ownRoadS; 
            }
            else
            {
                dist = ownCurState.ownRoadS; 
            }
        }
        else
        {
            std::list<KeyPoint>::iterator j = route_list.begin();
            for (; j != i; ++j )
            {
                if (roadNow->mId == j->curRoad->mId)
                    break;
            }
            if (i == j)
            {
                dist += roadNow->mLength; 
            }
        }


        // an old road, update distance would suffice.
        if ( i != route_list.end() )
        {
            i->curLane = laneNow;
            //std::cerr << "/*update old keypoint*/"<< std::endl;
            if ( laneNow->mId < 0 )
            {
                // if it is not the last laneSection
                if ( i->type )
                {
                	//std::cerr << "==hby== "<< std::endl;
                    getNextLaneSec( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, 0 );
                    
                    // there is a mal-section of length 1.0m in hby. <1.1 in order to skip it.
                    //if ( i->distance - dist + ( roadNow->mLength - laneSecNow->mSEnd ) < 1.1 )   
                    //{
                    //    std::cerr << "==hby== "<< std::endl;
                    //    break;
                    //}

    
                }
    
                else
                {
                    ++i;
                    if (i != route_list.end() )
                    {
                        int tarRoad = i->curRoad->mId;
                        --i;
                        getNextRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, i->roadDir, tarRoad );
                    }
                    else
                    {
                        --i;
                        getNextRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, i->roadDir, nextRoadId );
                    }

                    
                }
            
                //std::cerr << " dist: "<< dist << std::endl;
                //std::cerr << " ( roadNow->mLength - laneSecNow->mSEnd ) : "<<  roadNow->mLength - laneSecNow->mSEnd  << std::endl;
                i->distance = dist - ( roadNow->mLength - laneSecNow->mSEnd );
            }
            else
            {
                if ( i->type )
                {
                    getReverseNextLaneSec( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, 0);
                    
                    // there is a mal-section of length 1.0m in hby. <1.1 in order to skip it.
                    //if ( i->distance - dist + ( roadNow->mLength - laneSecNow->mSEnd ) < 1.1 )   
                    //{
                    //    std::cerr << "==hby== "<< std::endl;
                    //    break;
                    //}
    
                }
    
                else
                {
                    ++i;
                    if (i != route_list.end() )
                    {
                        int tarRoad = i->curRoad->mId;
                        --i;
                        getLastRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, i->roadDir, tarRoad );
                    }
                    else
                    {
                        --i;
                        getLastRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, i->roadDir, nextRoadId );
                    }
                }
            
                //std::cerr << " dist: "<< dist << std::endl;
                //std::cerr << " ( roadNow->mLength - laneSecNow->mSEnd ) : "<<  roadNow->mLength - laneSecNow->mSEnd  << std::endl;
                i->distance = dist - laneSecNow->mS;                
            } 

            //std::cerr << "00703roadNow:" << roadNow->mId << std::endl;
            //std::cerr << "00703laneNow:" << laneNow->mId << std::endl;
            //std::cerr << "00703roadNext:" << roadNext->mId << std::endl;
            //std::cerr << "00703laneNext:" << laneNext->mId << std::endl;

            ++i;


        }   

        // a new keypoint, create a new entry and push into list.
        else 
        {
            KeyPoint keypoint;

            int random_index = -2;

            // lane Ego currently on if dont lanechange
            keypoint.curLane = laneNow;
            // std::cerr << "0703roadNow:" << roadNow->mId << std::endl;
            // std::cerr << "laneSecNowS:" << laneSecNow->mS << std::endl;
            // std::cerr << "laneNow:" << laneNow->mId << std::endl;
            keypoint.curLaneSec = laneSecNow;
            if (laneNow)
            {
            	keypoint.drvDir = fabs(laneNow->mId) / laneNow->mId;
            }

            while ( !keypoint.nextLanes.size() && ++random_index < 3)
            {
                
                if  (laneNow->mId < 0) 
                {
                	// type
                    while( laneSecNow->getRight() )
                    {
                        //std::cerr << "in" << std::endl;
                        getNextLaneSec( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, 0);

                        if (!laneNext || !ifStrictlyDriveable(laneNext))
                        {
                        	// lane more to less
                            laneNext = 0;
                            //std::cerr << "lane more to less" << std::endl;
                        }

                        //std::cerr << "out" << std::endl;
                        //std::cerr << "roadNext:" << roadNext->mId << std::endl;
                        //std::cerr << "roadNext:" << roadNext << std::endl;
                        if ( roadNext == 0 || laneSecNext == 0)
                        {
                            //std::cerr << "123321" << std::endl;
                            return;
                        }
                        if ( roadNow->mId != roadNext->mId )
                        {
                        	//std::cerr << "roadNext:" << roadNext->mId << std::endl;
                            //std::cerr << "I'm out: " << roadNow->mId << std::endl;
                            break;
                        }


    
                        //std::cerr << "type of Keypoints" << std::endl;
                        int laneNumNow = 0;     // to calculate driving lane number of laneSecNow
                        int laneNumNext = 0;    // to calculate driving lane number of laneSecNext
    
                        for( int numCount = -1; numCount > -10; numCount-- )   
                        {

                            OpenDrive::Lane* laneNowForCount = laneSecNow->getLaneFromId(numCount);
                            
                            if(laneNowForCount)
                            {
                                if(ifStrictlyDriveable(laneNowForCount))
                                {
                                    laneNumNow++;
                                }
                            }

                            OpenDrive::Lane* laneNextForCount = laneSecNext->getLaneFromId(numCount);
                            if(laneNextForCount)
                            {
                                if(ifStrictlyDriveable(laneNextForCount))
                                {
                                    laneNumNext++;
                                }
                            }                        
    
                        }
                        //fprintf(stderr,"laneNumNow1:%i laneNumNext1:%i \n",laneNumNow,laneNumNext);
                        if(laneNumNext > laneNumNow)
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mSEnd;
                            keypoint.type = 2;
                            break;
                        }
                        else if(laneNumNext < laneNumNow)
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mSEnd;
                            keypoint.type = 1;
                            break;
                        }
    
                        //type 3
                        //else if ( laneNext->getFirstRoadMark() && laneNext->getFirstRoadMark()->mType == 1 && laneSecNext->mS == roadNext->getLastLaneSection()->mS )
                        else if ( laneNext && laneNext->getFirstRoadMark() )
                        {
                            OpenDrive::RoadMark* thisRoadMark = laneNext->getFirstRoadMark();
                            while( thisRoadMark->mType != 1 )
                            {
                                if ( thisRoadMark->getRight() )
                                {
                                    thisRoadMark = reinterpret_cast< OpenDrive::RoadMark* >( thisRoadMark->getRight() );
                                }
                                else
                                {
                                    break;
                                }
                            }
                            // 0317 zy: only if it is before a junction, then it is solid white -> which we need to calc roadS differently
                            // 0318 zy: close for now. ger2018 road115 s550
                            if ( 0 && thisRoadMark->mType == 1 && !roadNow->getSuccessor())
                            {
                                keypoint.roadDir = 0;
                                keypoint.roadS = laneSecNext->mS + laneNext->getFirstRoadMark()->mOffset;
                                keypoint.type = 3;
        
                                //whether it is valid
                                if ( int(keypoint.roadS) % 100 > solidObeyCoef * 100 )
                                {
                                    keypoint.valid = 0;
                                }
                            }
                            else
                            {
                                keypoint.roadDir = 0;
                                keypoint.roadS = laneSecNow->mSEnd;
                                keypoint.type = 4;
                            }
                            break;

                        }
                        // type 4
                        else
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mSEnd;
                            keypoint.type = 4;
                            break;
                        }
                        
                    }

                    // distance
                    if ( keypoint.distance == -100 )
                    {
                        keypoint.distance = dist - ( roadNow->mLength - laneSecNow->mSEnd );
                    }
                    
                    if ( !keypoint.type )
                    {
                        //std::cerr << "0609type=0"<< std::endl;
                        //std::cerr << "roadNow: " << roadNow->mId << std::endl;
                        //std::cerr << "laneNow: " << laneNow->mId << std::endl;


                        if ( roadNow->mId == nextRoadId )
                        {
                            nextRoadId = 0;
                        }


                        //20200609 add route_paths
                        if ( ownCurState.nextRoadIdx < route_paths.size() )
                        {
                            nextRoadId = route_paths[ownCurState.nextRoadIdx];
                            ++ownCurState.nextRoadIdx;
                        }



                        keypoint.roadS = roadNow->mLength;
                        keypoint.roadDir = getNextRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, (random_index + int(ownCurState.ownRoadS) + retry) % 4- 1 , nextRoadId );
                                                
                        if (roadNext)
                        {
                        	nextRoadId = roadNext->mId;
                        }
                        else
                        {
                            //end of the world
                            std::cerr << "end of the world ~ " << std::endl;
                            keypoint.curRoad = roadNow;
                            end_of_the_world = roadNow->mId;
                            keypoint.distance = roadNow->mLength - ownCurState.ownRoadS;
                            keypoint.type = 0;
                            keypoint.velMax = 0.0;
                            //std::cerr << "ownCurState.drvDir:" << ownCurState.drvDir << std::endl;

                            int laneNo = -11;
                            while ( ++laneNo < 11)
                            {
                                if ( laneSecNow->getLaneFromId(laneNo) )
                                {
                                    OpenDrive::Lane* thisLane = laneSecNow->getLaneFromId(laneNo);
                                    if (!ifStrictlyDriveable(thisLane) || thisLane->mId * ownCurState.drvDir < 0)
                                    {
                                        continue;
                                    }
            
                                    keypoint.nextLanes.push_back(thisLane);
                                }
                            }

                            route_list.push_back(keypoint); 
                            return;
                        }

                        if (!laneSecNext || !laneNext)
                        {
                        	//std::cerr << "&&&laneSecNext: "<< laneSecNext << std::endl;
                        	//std::cerr << "&&&laneNext: "<< laneNext << std::endl;
                        	break;
                        }                        

                    }

                }

                else
                {
                    //std::cerr << "roadNow123: "<< roadNow->mId << std::endl;
                    //std::cerr << "laneNow123: "<< laneNow->mId << std::endl;
                    while( laneSecNow->getLeft() )
                    {
                        getReverseNextLaneSec( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, 0);

                        if (!laneNext || !ifStrictlyDriveable(laneNext))
                        {
                            // lane more to less
                            laneNext = 0;
                            //std::cerr << "lane more to less" << std::endl;
                        }
                        //std::cerr << "out" << std::endl;
                        //std::cerr << "roadNext:" << roadNext->mId << std::endl;
                        //std::cerr << "roadNext:" << roadNext << std::endl;
                        if ( roadNext == 0 || laneSecNext == 0 )
                        {
                            //std::cerr << "123321" << std::endl;
                            return;
                        }
                        if ( roadNow->mId != roadNext->mId )
                        {
                            //std::cerr << "roadNext:" << roadNext->mId << std::endl;
                            //std::cerr << "I'm out: " << roadNow->mId << std::endl;
                            break;
                        }
                             

                        int laneNumNow = 0;     // to calculate driving lane number of laneSecNow
                        int laneNumNext = 0;    // to calculate driving lane number of laneSecNext
    
                        for( int numCount = 0; numCount < 10; numCount++ )
                        {
                            OpenDrive::Lane* laneNowForCount = laneSecNow->getLaneFromId(numCount);
                            
                            if(laneNowForCount)
                            {
                                if(ifStrictlyDriveable(laneNowForCount) ) 
                                {
                                    laneNumNow++;
                                }
                            }
                            OpenDrive::Lane* laneNextForCount = laneSecNext->getLaneFromId(numCount);
                            if(laneNextForCount)
                            {
                                if(ifStrictlyDriveable(laneNextForCount) )
                                {
                                    laneNumNext++;
                                }
                            }                        
    
                        }
                        //fprintf(stderr,"laneNumNow2:%i laneNumNext2:%i \n",laneNumNow,laneNumNext);
                        if(laneNumNext > laneNumNow)
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mS;
                            keypoint.type = 2;
                            break;
                        }
                        else if(laneNumNext < laneNumNow)
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mS;
                            keypoint.type = 1;
                            break;
                        }
    
                        //type 3
                        //if ( laneNext->getFirstRoadMark() && laneNext->getFirstRoadMark()->mType == 1 && laneSecNext->mS == roadNext->getFirstLaneSection()->mS )
                        else if ( laneNext && laneNext->getFirstRoadMark() )
                        {
                            OpenDrive::RoadMark* thisRoadMark = laneNext->getFirstRoadMark();
                            while( thisRoadMark->mType != 1 )
                            {
                                if ( thisRoadMark->getRight() )
                                {
                                    thisRoadMark = reinterpret_cast< OpenDrive::RoadMark* >( thisRoadMark->getRight() );
                                }
                                else
                                {
                                    break;
                                }
                            }
                            // 0317 zy: only if it is before a junction, then it is solid white -> which we need to calc roadS differently
                            // 0318 zy: for security issue, close for now
                            if ( 0 && thisRoadMark->mType == 1 && !roadNow->getPredecessor())
                            {
                                keypoint.roadDir = 0;
                                keypoint.roadS = laneSecNext->mS + laneNext->getFirstRoadMark()->mOffset;
                                keypoint.type = 3;
        
                                //whether it is valid
                                if ( int(keypoint.roadS) % 100 > solidObeyCoef * 100 )
                                {
                                    keypoint.valid = 0;
                                }
                            }
                            else
                            {
                                keypoint.roadDir = 0;
                                keypoint.roadS = laneSecNow->mS;
                                keypoint.type = 4;
                            }
                            break;

                        }
                        // type 4
                        else
                        {
                            keypoint.roadDir = 0;
                            keypoint.roadS = laneSecNow->mS;
                            keypoint.type = 4;
                            break;
                        }
                        
                    }

                    // distance
                    if ( keypoint.distance == -100 )
                    {
                        keypoint.distance = dist - laneSecNow->mS;
                    }
                    
                    if ( !keypoint.type )
                    {
                        //std::cerr << "yes"<< std::endl;
                        if ( roadNow->mId == nextRoadId )
                        {
                            nextRoadId = 0;
                        }

                        //20200609 add route_paths
                        if ( ownCurState.nextRoadIdx < route_paths.size() )
                        {
                            nextRoadId = route_paths[ownCurState.nextRoadIdx];
                            ++ownCurState.nextRoadIdx;
                        }

                        keypoint.roadS = 0;

                        keypoint.roadDir = getLastRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, (random_index + int(ownCurState.ownRoadS) + retry) % 4- 1, nextRoadId );
                        
                        if (roadNext)
                        {
                        	nextRoadId = roadNext->mId;
                        }
                        else
                        {
                            //end of the world
                            std::cerr << "end of the world ~ " << std::endl;
                            keypoint.curRoad = roadNow;
                            end_of_the_world = roadNow->mId;
                            keypoint.type = 0;
                            keypoint.distance = roadNow->mLength - ownCurState.ownRoadS;
                            keypoint.velMax = 0.0;
                            //std::cerr << "ownCurState.drvDir:" << ownCurState.drvDir << std::endl;

                            int laneNo = -11;
                            while ( ++laneNo < 11)
                            {
                                if ( laneSecNow->getLaneFromId(laneNo) )
                                {
                                    OpenDrive::Lane* thisLane = laneSecNow->getLaneFromId(laneNo);
                                    if (!ifStrictlyDriveable(thisLane) || thisLane->mId * ownCurState.drvDir < 0)
                                    {
                                        continue;
                                    }
            
                                    keypoint.nextLanes.push_back(thisLane);
                                }
                            }

                            route_list.push_back(keypoint); 
                            return;
                        }

                        if (!laneSecNext || !laneNext)
                        {
                        	//std::cerr << "&&&laneSecNext: "<< laneSecNext << std::endl;
                        	//std::cerr << "&&&laneNext: "<< laneNext << std::endl;
                        	break;
                        }
                        
                    }


                }

                // road
                keypoint.curRoad = roadNow;

                std::cerr << "========= "<< std::endl;
                std::cerr << "roadNow: "<< roadNow->mId<< std::endl;
                std::cerr << "laneNow: "<< laneNow->mId<< std::endl;
                std::cerr << "roadNext: "<< roadNext->mId<< std::endl;
                if (laneNext)
                {
                    std::cerr << "laneNext: "<< laneNext->mId<< std::endl;
                }
                
                std::cerr << "========= "<< std::endl;

                // 0324: find right laneNext before find nextLanes.
                // same thing down below, for update old routeList.
                // for curLane. Find nearest lane
                if (!laneNext)
                {
                    int drvDir = laneNow->mId;
                    OpenDrive::Lane* tmpLane = laneNow;
                    int cnt = 0;
                    while ( tmpLane->getLeft() )
                    {
                        ++cnt;
                        tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getLeft() );
                        //0322: Dont cross mid line
                        if (!ifStrictlyDriveable(tmpLane) || drvDir * tmpLane->mId < 0)
                        {
                            std::cerr << "0322 Dont cross mid line" <<std::endl;
                            break;
                        }
                        getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
                        if (laneNext && ifStrictlyDriveable(laneNext))
                        {
                            break;
                        }
                    }

                    if (!laneNext)
                    {
                        while(cnt--)
                        {
                            tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getRight() );
                        }
                        while ( tmpLane->getRight() )
                        {
                            tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getRight() );
                            //std::cerr << "0322tmpLane->mId" << tmpLane->mId<<std::endl;
                            getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
                            if (laneNext && ifStrictlyDriveable(laneNext))
                            {
                                break;
                            }
                        }    
                    }
                    std::cerr << "0322laneNext->mId" << laneNext->mId<<std::endl;
                }
                if (!laneNext)
                {
                    std::cerr << "0324 deal with it! "<< std::endl;
                }

                /* nextLanes */
                if (laneNext)
                {
                    if (laneNext->mId < 0)
                    {
                        int laneNo = -11;
                        while ( ++laneNo < 0)
                        {
                            if ( laneSecNext->getLaneFromId(laneNo) )
                            {
                                OpenDrive::Lane* thisLane = laneSecNext->getLaneFromId(laneNo);

                                if (!ifStrictlyDriveable(thisLane))
                                {
                                    continue;
                                }

                                OpenDrive::RoadHeader* roadLast = 0;
                                OpenDrive::LaneSection* laneSecLast = 0;
                                OpenDrive::Lane* laneLast = 0;

                                if ( getLastLaneEntry( roadNext, laneSecNext, thisLane, roadLast, laneSecLast, laneLast, roadNow->mId) )
                                {
                                    if ( laneLast && ifStrictlyDriveable(laneLast) )
                                    {
                                        keypoint.nextLanes.push_back(laneLast);
                                    
                                    }
                                    
                                }

                            }
                        }                        
                    }
                    else
                    {
                        int laneNo = 1;
                        while ( ++laneNo < 10)
                        {
                            if ( laneSecNext->getLaneFromId(laneNo) )
                            {
                                OpenDrive::Lane* thisLane = laneSecNext->getLaneFromId(laneNo);
                                
                                if (!ifStrictlyDriveable(thisLane))
                                //if ( thisLane->mType != 1 && thisLane->mType != 7 && thisLane->mType != 21 && thisLane->mType != 0 )
                                {
                                    continue;
                                }

                                OpenDrive::RoadHeader* roadLast = 0;
                                OpenDrive::LaneSection* laneSecLast = 0;
                                OpenDrive::Lane* laneLast = 0;

                                if ( getLastLaneEntry2( roadNext, laneSecNext, thisLane, roadLast, laneSecLast, laneLast, roadNow->mId) )
                                {
                                    if ( laneLast && ifStrictlyDriveable(laneLast) )
                                    {
                                        keypoint.nextLanes.push_back(laneLast);
                                    }
                                    
                                } 
            
                            }
                        }
                    }                    
                }

                //0324 if cannot find any, then push Now into, for robustness
                if (keypoint.nextLanes.empty())
                {
                    keypoint.nextLanes.push_back(laneNow);
                }

                for (std::list<OpenDrive::Lane*>::iterator j = keypoint.nextLanes.begin(); j != keypoint.nextLanes.end(); ++j)
                {
                    if ((*j) && (*j)->mId) 
                        std::cerr << "0324nextLaneId: "<< (*j)->mId << std::endl;
                }

                //20200819 zy added: only save those closest to ownlane
                if (laneNow)
                {
                    trimNextLanes(keypoint.nextLanes, laneNow->mId);
                }

            }

            double curvature;
            if ( keypoint.type == 0 )
            {
                //if (laneNext->mId < 0 )
                if ( (laneNext && laneNext->mId < 0) || laneSecNext->getLaneFromId(-1))
                {
                    curvature = fabs(roadNext->getMaxCurvatureInRange( 0, 20 ));
                }
                else
                {
                    curvature = fabs(roadNext->getMaxCurvatureInRange( roadNext->mLength - 20, roadNext->mLength ));
                }
            }
            else if ( keypoint.type == 1 || keypoint.type == 3 || keypoint.type == 4 )
            {
                //if (laneNext->mId < 0 )
                if ( (laneNext && laneNext->mId < 0) || laneSecNext->getLaneFromId(-1))
                {
                    if(keypoint.roadS + 20 < roadNow->mLength)
                        curvature = fabs(roadNow->getMaxCurvatureInRange(keypoint.roadS, keypoint.roadS + 20));
                    else
                        curvature = fabs(roadNow->getMaxCurvatureInRange(keypoint.roadS, roadNow->mLength));
                }
                else
                {
                    if( keypoint.roadS - 20 > 0 )
                        curvature = fabs(roadNow->getMaxCurvatureInRange(keypoint.roadS - 20, keypoint.roadS));
                    else
                        curvature = fabs(roadNow->getMaxCurvatureInRange(0, keypoint.roadS));
                }
            }
            //else if ( keypoint.type == 3 )
            //{
            //    if(keypoint.roadS + 20 < roadNow->mLength)
            //    {
            //        curvature = fabs(roadNow->getMaxCurvatureInRange(keypoint.roadS, keypoint.roadS + 20));
            //        //fprintf(stderr,"curvature:%.3f roadS:%.3f roadS+:%.3f\n",curvature,keypoint.roadS,keypoint.roadS + 20);
            //    }
            //    else
            //        curvature = fabs(roadNow->getMaxCurvatureInRange(keypoint.roadS, roadNow->mLength));
            //}
            //std::cerr << "curvature: "<< curvatures << std::endl;
            if (curvature < 1.0/500)
            {
                keypoint.velMax = 120.0/3.6;    //WARNING
            }
            else
            {
                double r = 1.0/curvature;
                //std::cerr << "r: "<< r << std::endl;
                keypoint.velMax = calCurVel(fabs(r));
            }

            // before push, modify the last one's lane choices if this one is type 0 and last one is type 2
            if ( keypoint.type == 0 )
                if ( route_list.size() && route_list.back().type == 3 )
                {
                    route_list.back().nextLanes = keypoint.nextLanes;
                }

            // 0322: before push, if curLane is wierd, change curLane
            // if (keypoint.curLane)

            route_list.push_back(keypoint); 
        
        }

        // robust
		if ( !roadNext || !laneSecNext )
		{
			if( !route_list.size() || route_list.back().distance < 10 || !roadNow || !roadNow->mId )
			{
				// stop now
				std::cerr << "launch safe mode!(stop now)" << std::endl;
    			return;
			}
			if( retry > 4 )
			{
				if (!roadNext)
				{
					// stop later
					std::cerr << "launch safe mode!(no way out)" << std::endl;
					return;
				}
				else
				{
					// leap of fate
					std::cerr << "launch safe mode!(go with dare)" << std::endl;
					return;
				}

			}
			else 
			{
				// retry
				retry += 1;
				return;
			}
		}
		else
		{
			retry = 0;
		}

        // for curLane. Find nearest lane
        if (!laneNext)
        {
            std::cerr << "0324444" <<std::endl;
            int drvDir = laneNow->mId;
            OpenDrive::Lane* tmpLane = laneNow;
            int cnt = 0;
            while ( tmpLane->getLeft() )
            {
                ++cnt;
                tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getLeft() );
                //0322: Dont cross mid line
                if (!ifStrictlyDriveable(tmpLane) || drvDir * tmpLane->mId < 0)
                {
                    std::cerr << "0322 Dont cross mid line" <<std::endl;
                    break;
                }
                getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
                if (laneNext && ifStrictlyDriveable(laneNext))
                {
                    break;
                }
            }

            if (!laneNext)
            {
                while(cnt--)
                {
                    tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getRight() );
                }
                while ( tmpLane->getRight() )
                {
                    tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getRight() );
                    //std::cerr << "0322tmpLane->mId" << tmpLane->mId<<std::endl;
                    getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
                    if (laneNext && ifStrictlyDriveable(laneNext))
                    {
                        break;
                    }
                }    
            }
            std::cerr << "0322laneNext->mId" << laneNext->mId<<std::endl;
        }

        // OLD // for curLane. Find nearest lane
        // if (!laneNext)
        // {
        //     int drvDir = laneNow->mId;
        //     OpenDrive::Lane* tmpLane = laneNow;
        //     while ( tmpLane->getLeft() )
        //     {
        //         tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getLeft() );
        //         //0322: Dont cross mid line
        //         if (!ifStrictlyDriveable(tmpLane) || drvDir * tmpLane->mId < 0)
        //         {
        //             std::cerr << "0322 Dont cross mid line" <<std::endl;
        //             break;
        //         }
        //         getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
        //         if (laneNext && ifStrictlyDriveable(laneNext))
        //         {
        //             break;
        //         }
        //     }

        //     if (!laneNext)
        //     {
        //         tmpLane = laneNow;
        //         while ( tmpLane->getRight() )
        //         {
        //             tmpLane = reinterpret_cast< OpenDrive::Lane* >( tmpLane->getRight() );
        //             //std::cerr << "0322tmpLane->mId" << tmpLane->mId<<std::endl;
        //             getNextLaneSec( roadNow, laneSecNow, tmpLane, roadNext, laneSecNext, laneNext, 0);
        //             if (laneNext && ifStrictlyDriveable(laneNext))
        //             {
        //                 break;
        //             }
        //         }    
        //     }
        //     std::cerr << "0322laneNext->mId" << laneNext->mId<<std::endl;
        // }

        roadNow = roadNext;
        laneNow = laneNext;
        laneSecNow = laneSecNext;

        roadNext = 0;
        laneNext = 0;
        laneSecNext = 0;        
    }

    return;	
}

int
RouteList::getNextRoad( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadNext, OpenDrive::LaneSection* &laneSecNext, OpenDrive::Lane* &laneNext, int roadDir, int tarRoadId)
{
    //If next lanesection exist, return it rather than next road.
    while ( laneSecNow->getRight() && laneNow->getSuccessor() )
    {
        //std::cerr << "In 1" << std::endl;
        laneSecNow = reinterpret_cast<OpenDrive::LaneSection*>(laneSecNow->getRight());

        //std::cerr << "In 2" << std::endl;
        //std::cerr << "laneNow->getSuccessor()->mId: " << laneNow->getSuccessor()->mId << std::endl;
        laneNow = laneNow->getSuccessor();   
        
        //if (roadNext->mId)
        //    std::cerr << "1RoadNextId: " << roadNext->mId << std::endl;
        //if (laneNext->mId)
        //    std::cerr << "1LaneNextId: " << laneNext->mId << std::endl;

        //std::cerr << "3" << std::endl;
        
        
        //std::cerr << "1end" << std::endl;
    }

    // get successor if existed
    
    if ( roadNow->getSuccessor() && laneNow->getSuccessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadNext = roadNow->getSuccessor();
        laneSecNext = roadNext->getFirstLaneSection();

        laneNext = laneNow->getSuccessor();
        if (laneNext->getFirstWidth() && laneNext->mId >= 0)
        {
            laneSecNext = roadNext->getLastLaneSection();
        }
        
        std::cerr << "1RoadNextId: " << roadNext->mId << std::endl;
        std::cerr << "laneSecNexts: " << laneSecNext->mS << std::endl;
        std::cerr << "laneSecNextSend: " << laneSecNext->mSEnd << std::endl;

        return 0;
        
    }

    else
    {   
        //std::cerr << "In 3" << std::endl;
        OpenDrive::OdrManager myManager;
        // create a pos object and activate it
        OpenDrive::Position* myPos;
        myPos = myManager.createPosition();
        myManager.activatePosition(myPos);
        
        myManager.setLanePos(roadNow->mId, laneNow->mId, 0, 0);

        OpenDrive::JuncHeader *juncHdr = 0;
        double distance;
        bool retVal = myManager.getNextJunction( true, juncHdr, distance );
        
        
        if (retVal && juncHdr)
        {
            double angle;
            int minLaneIdIncoming;
            int maxLaneIdIncoming;
            OpenDrive::RoadHeader* roadtmp = 0;
            int junctionInfoSize = myManager.getJunctionInfoSize();
            int round2 = 0;
            // decide which road to go
            for ( int i = 0; i <= junctionInfoSize*2 - 1; i++ )
            {
                //std::cerr << "i: "<< i << "  tarRoadId: " << tarRoadId << std::endl;
                if (i > junctionInfoSize - 1)
                {
                    if (i == junctionInfoSize)
                    {
                        round2 += 1;
                    }
                    i -= junctionInfoSize;                    
                }
                //std::cerr << "round2: "<< round2 << std::endl;

                myManager.getJunctionInfo( i, roadtmp, roadNext, angle, minLaneIdIncoming, maxLaneIdIncoming );
                //if ( tarRoadId != 0 )
                //{
                //    std::cerr << "RoadNowIdJ: " << roadtmp->mId 
                //              << "  RoadNextIdJ: " << roadNext->mId 
                //              << "  tarRoadId: " << tarRoadId
                //              << "  angle:  " << angle 
                //              << "  leng:  " << roadNext->mLength << std::endl;
                //}
                //std::cerr << "getNoDriveableLanes: "<< roadNext->getNoDriveableLanes() << std::endl;
                if ( tarRoadId != 0 )
                {
                    if ( roadNext->mId == tarRoadId || round2 > 1 )
                    {
                        if (round2 > 1)
                        {
                            std::cerr << "round2 > 1" << std::endl;
                        }
                        else
                        {
                            std::cerr << "decide road" << std::endl;
                        }
                        // 0318
                        if (laneNow->mId > 0)
                        {
                            laneSecNext = roadNext->getLastLaneSection();
                        }
                        else
                        {
                            laneSecNext = roadNext->getFirstLaneSection();
                        }

                        roadDir = int(angle*0.7);
                    }
                    else
                    {
                        continue;
                    }                    
                }

                else if (  roadNext && roadNext->getNoDriveableLanes() && roadtmp->mId == roadNow->mId )
                {
                    //std::cerr << "not decide road" << std::endl;
                    //std::cerr << "roadNext->mId: " << roadNext->mId << std::endl;
                    //find the road with right roadDir
                    if ( fabs( roadDir - (angle*0.7) ) <= 0.5 || round2 )
                    {
                        //std::cerr << "roadNext->mId: " << roadNext->mId << std::endl;
                        //roadNext = roadtmp2;
                        
                        // 0318
                        if (laneNow->mId > 0)
                        {
                            laneSecNext = roadNext->getLastLaneSection();
                        }
                        else
                        {
                            laneSecNext = roadNext->getFirstLaneSection();
                        }
                        if ( round2 )
                        {
                            roadDir = int(angle*0.7);
                        }
                    }
                    else
                    {
                        continue;
                    }

                }

                OpenDrive::JuncLink* jclink = juncHdr->getFirstLink();
    
                if ( !roadNow->getFirstGeoHeader() || !roadNext->getFirstGeoHeader() )
                {
                    return roadDir;
                }

                // find the right junction link
                while ( jclink->mIncomingRoad->mId != roadNow->mId || jclink->mConnectingRoad->mId != roadNext->mId )
                {
                    //std::cerr << "jclink->mConnectingRoad->mId: " << jclink->mConnectingRoad->mId << std::endl;
                    if ( jclink->getRight() )
                    {
                        jclink =  reinterpret_cast< OpenDrive::JuncLink* >( jclink->getRight() );
                    }
                    else
                    {
                        return roadDir;
                    }
                }

                // find the right junction lanelink
                OpenDrive::JuncLaneLink* jclanelink = jclink->getFirstLaneLink();

                int running_cnt = maxLaneIdIncoming - minLaneIdIncoming;
                while ( jclanelink != NULL && running_cnt >= 0 )
                {    
                    if ( jclanelink->mConnectingLane != NULL && jclanelink->mIncomingLane->mId <= maxLaneIdIncoming && jclanelink->mIncomingLane->mId >= minLaneIdIncoming && ifDriveable(jclanelink->mConnectingLane) )
                    {
                        if ( jclanelink->mIncomingLane->mId == laneNow->mId || round2 > 1 )
                        {
                            //std::cerr << "round2: "<< round2 << std::endl;
                            laneNext = jclanelink->mConnectingLane;
                            return roadDir;
                        }                        

                    }
                    jclanelink =  reinterpret_cast< OpenDrive::JuncLaneLink* >( jclanelink->getRight() );
                    --running_cnt;
                }
            }
        }
    }
    return roadDir;
}


int
RouteList::getLastRoad( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadNext, OpenDrive::LaneSection* &laneSecNext, OpenDrive::Lane* &laneNext, int roadDir, int tarRoadId)
{
    //If next lanesection exist, return it rather than next road.
    while ( laneSecNow->getLeft() && laneNow->getPredecessor() )
    {
        //std::cerr << "In 1" << std::endl;
        laneSecNow = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );

        //std::cerr << "laneNow->getSuccessor()->mId: " << laneNow->getSuccessor()->mId << std::endl;
        laneNext = laneNow->getPredecessor();   

        //if (roadNext->mId)
        //    std::cerr << "1RoadNextId: " << roadNext->mId << std::endl;
        //if (laneNext->mId)
        //    std::cerr << "1LaneNextId: " << laneNext->mId << std::endl;

        //std::cerr << "3" << std::endl;
        
        
        //std::cerr << "1end" << std::endl;
    }
    // get successor if existed
    
    if ( roadNow->getPredecessor() && laneNow->getPredecessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadNext = roadNow->getPredecessor();
        laneSecNext = roadNext->getLastLaneSection();

        laneNext = laneNow->getPredecessor();
        if (laneNext->getFirstWidth() && laneNext->mId < 0)
        {
            laneSecNext = roadNext->getFirstLaneSection();
        }
        
        std::cerr << "2RoadNextId: " << roadNext->mId << std::endl;
        std::cerr << "laneSecNexts: " << laneSecNext->mS << std::endl;
        std::cerr << "laneSecNextSend: " << laneSecNext->mSEnd << std::endl;
        
        if (laneNext == 0)
        {
        
        }
        //else
        //{
        //    std::cerr << "2RoadNextId: " << roadNext->mId << std::endl;
        //    std::cerr << "2LaneNextId: " << laneNext->mId << std::endl;
        //}
        return 0;
        
    }

    else
    {   
        //std::cerr << "0318In 3" << std::endl;
        OpenDrive::OdrManager myManager;
        // create a pos object and activate it
        OpenDrive::Position* myPos;
        myPos = myManager.createPosition();
        myManager.activatePosition(myPos);
        
        myManager.setLanePos(roadNow->mId, laneNow->mId, 0, 0);

        OpenDrive::JuncHeader *juncHdr = 0;
        double distance;
        bool retVal = myManager.getNextJunction( false, juncHdr, distance );
        
        
        if (retVal && juncHdr)
        {
            double angle;
            int minLaneIdIncoming;
            int maxLaneIdIncoming;
            OpenDrive::RoadHeader* roadtmp;
            int junctionInfoSize = myManager.getJunctionInfoSize();
            int round2 = 0;

            // decide which road to go
            for ( int i = 0; i <= junctionInfoSize*2 - 1; i++ )
            {
                if (i > junctionInfoSize - 1)
                {
                    if (i == junctionInfoSize)
                    {
                        round2 += 1;
                    }
                    i -= junctionInfoSize;                    
                }

                myManager.getJunctionInfo( i, roadtmp, roadNext, angle, minLaneIdIncoming, maxLaneIdIncoming );
                //if (roadNext)
                //{
                //    std::cerr << "RoadNextIdJ: " << roadNext->mId 
                //              << "     angle:  " << angle 
                //              << "      leng:  " << roadNext->mLength << std::endl;
                //}
                if ( tarRoadId != 0 )
                {
                    if ( roadNext->mId == tarRoadId || round2 > 1 )
                    {
                        std::cerr << "decide road" << std::endl;
                        // 0318
                        if (laneNow->mId < 0)
                        {
                            laneSecNext = roadNext->getLastLaneSection();
                        }
                        else
                        {
                            laneSecNext = roadNext->getFirstLaneSection();
                        }
                        roadDir = int(angle*0.7);
                    }
                    else
                    {
                        continue;
                    }                    
                }                

                else if ( roadNext && roadNext->getNoDriveableLanes() && roadtmp->mId == roadNow->mId )
                {
                    //std::cerr << "roadNext->mId: " << roadNext->mId << std::endl;
                    //find the road with right roadDir
                    if ( fabs(roadDir - (angle*0.7) ) <= 0.5 || round2)
                    {
                        // 0318
                        if (laneNow->mId < 0)
                        {
                            laneSecNext = roadNext->getLastLaneSection();
                        }
                        else
                        {
                            laneSecNext = roadNext->getFirstLaneSection();
                        }
                        if ( round2 )
                        {
                            roadDir = int(angle*0.7);
                        }
                    }
                    else
                    {
                        continue;
                    }

                }   

                OpenDrive::JuncLink* jclink = juncHdr->getFirstLink();
    
                if ( !roadNow->getFirstGeoHeader() || !roadNext->getFirstGeoHeader())
                {
                    return roadDir;
                }
    
                //std::cerr << "roadNext->mId: " << roadNext->mId << std::endl;
                while ( jclink->mIncomingRoad->mId != roadNow->mId || jclink->mConnectingRoad->mId != roadNext->mId )
                {
                    //std::cerr << "jclink->mConnectingRoad->mId: " << jclink->mConnectingRoad->mId << std::endl;
                    if ( jclink->getRight() )
                        jclink =  reinterpret_cast< OpenDrive::JuncLink* >( jclink->getRight() );
                    else
                        return roadDir;
                }
                //std::cerr << "inRoad: "<< jclink->mIncomingRoad->mId << std::endl; 
    
                OpenDrive::JuncLaneLink* jclanelink = jclink->getFirstLaneLink();
                
                //std::cerr << "ownCurState.ownLaneId: "<< ownCurState.ownLaneId << std::endl;
                int running_cnt = maxLaneIdIncoming - minLaneIdIncoming;
                while ( jclanelink != NULL && running_cnt >= 0 )
                {
    
                    //std::cerr << "inLane: "<< jclanelink->mIncomingLane->mId << std::endl;
                    //std::cerr << "outLane: "<< jclanelink->mConnectingLane->mId << std::endl;
    
                    if ( jclanelink->mConnectingLane != NULL && jclanelink->mIncomingLane->mId <= maxLaneIdIncoming && jclanelink->mIncomingLane->mId >= minLaneIdIncoming && ifDriveable(jclanelink->mConnectingLane) )
                    {
                        if (jclanelink->mIncomingLane->mId == laneNow->mId || round2 > 1 )
                        {
                            laneNext = jclanelink->mConnectingLane;
                            return roadDir;
                        }                            

                    }
                    jclanelink =  reinterpret_cast< OpenDrive::JuncLaneLink* >( jclanelink->getRight() );
                    --running_cnt;
                }
                //std::cerr << "inLane: "<< jclanelink->mIncomingLane->mId << std::endl;
                //std::cerr << "outLane: "<< jclanelink->mConnectingLane->mId << std::endl;
                //OpenDrive::Lane* lane =  reinterpret_cast< OpenDrive::Lane* >( sec->getChild( OpenDrive::ODR_OPCODE_LANE ) );
            }
        }

        //if (roadNext->mId)
        //    std::cerr << "3RoadNextId: " << roadNext->mId << std::endl;
        //if (laneNext->mId)
        //    std::cerr << "3LaneNextId: " << laneNext->mId << std::endl;
    }
    return roadDir;


}


int 
RouteList::getLastLaneEntry( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadLast, OpenDrive::LaneSection* &laneSecLast, OpenDrive::Lane* &laneLast, int roadInId )
{
    //If last lanesection exist, return it rather than last road.
    if ( laneSecNow->getLeft() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        roadLast = roadNow;
        laneSecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        if ( laneNow->getPredecessor() )
        {
            laneLast = laneNow->getPredecessor();
        }
        else
        {
            return 0;
        }

        //std::cerr << "1RoadLastId: " << roadLast->mId << std::endl;
        //std::cerr << "1LaneLastId: " << laneLast->mId << std::endl;

        if ( roadLast->mId == roadInId )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    }

    // get predecessor if existed
    else if ( roadNow->getPredecessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadLast = roadNow->getPredecessor();
        laneSecLast = roadLast->getLastLaneSection();
        if ( laneNow->getPredecessor() )
        {
            laneLast = laneNow->getPredecessor();
        }
        else
        {
            return 0;
        }

        //if (laneLast == 0)
        //{
        //
        //}
        //else
        //{
        //    std::cerr << "2RoadLastId: " << roadLast->mId << std::endl;
        //    std::cerr << "2LaneLastId: " << laneLast->mId << std::endl;
        //}

        if ( roadLast->mId == roadInId )
        {
            return 1;
        }
        else
        {
            return 0;
        }

        
    }

    else
    {
        //std::cerr << "In 3" << std::endl;
        if (!roadNow->getPredecessorNode())
        {
            //std::cerr << "BOTW" << std::endl;
            return 0;
        }

        OpenDrive::JuncHeader* juncHdr = reinterpret_cast< OpenDrive::JuncHeader* >( roadNow->getPredecessorNode() );

        if ( !juncHdr->getFirstLink() )
        {
            return 0;
        }
        OpenDrive::JuncLink* jclink = juncHdr->getFirstLink();
        //std::cerr << "31" << std::endl;
        //while (jclink->mIncomingRoad->mId != 14011 || jclink->mConnectingRoad->getSuccessor() != roadNow)

        while ( (jclink->mConnectingRoad->getSuccessor() && jclink->mConnectingRoad->getSuccessor()->mId != roadNow->mId ) || (jclink->mConnectingRoad && jclink->mConnectingRoad->mId != roadInId) )
        {
            //std::cerr << "jclink->mConnectingRoad->getSuccessor()->mId:" << jclink->mConnectingRoad->getSuccessor()->mId << std::endl;
            //std::cerr << "jclink->mConnectingRoad->mId:" << jclink->mConnectingRoad->mId << std::endl;
            if ( jclink->getRight() )
            {
                jclink = reinterpret_cast< OpenDrive::JuncLink* >( jclink->getRight() );
            }
            else
            {
                return 0;
            }

        }


        //std::cerr << "roadfindout: " << jclink->mConnectingRoad->mId << std::endl;

        roadLast = jclink->mConnectingRoad;

        //std::cerr << "roadIn: " << roadInId << std::endl;
        //std::cerr << "roadfind: " << roadLast->mId << std::endl;

        laneSecLast = roadLast->getLastLaneSection();
        //std::cerr << "32" << std::endl;
        int cnt = 10;
        while (cnt > -10)
        {
            //std::cerr << "found lane suc"<< lsec->getLaneFromId(cnt)->getSuccessor()->mId << std::endl;
            if( laneSecLast->getLaneFromId(cnt) )
            {
                if ( laneSecLast->getLaneFromId(cnt)->getSuccessor() )
                {
                    //std::cerr << "good: "<< laneSecLast->getLaneFromId(cnt)->getSuccessor()->mId << std::endl;
                    if ( laneSecLast->getLaneFromId(cnt)->getSuccessor()->mId == laneNow->mId )
                    {
                        laneLast = laneSecLast->getLaneFromId(cnt);

                        //std::cerr << "3RoadLastId: " << roadLast->mId << std::endl;
                        //if (laneLast->mId)
                        //{
                        //    std::cerr << "3LaneLastId: " << laneLast->mId << std::endl;
                        //}

                        return 1;
                    }
                }

            }        
            --cnt;
        }
        //std::cerr << "3end" << std::endl;


        //std::cerr << "3RoadLastId: " << roadLast->mId << std::endl;
        //if (laneLast->mId)
        //{
        //    std::cerr << "3LaneLastId: " << laneLast->mId << std::endl;
        //}
        
        
    }


}

int 
RouteList::getLastLaneEntry2( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadLast, OpenDrive::LaneSection* &laneSecLast, OpenDrive::Lane* &laneLast, int roadInId )
{
    //If last lanesection exist, return it rather than last road.
    if ( laneSecNow->getRight() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );
        roadLast = roadNow;
        laneSecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );
        if ( laneNow->getSuccessor() )
        {
            laneLast = laneNow->getSuccessor();
        }
        else
        {
            return 0;
        }

        //std::cerr << "1RoadLastId: " << roadLast->mId << std::endl;
        //std::cerr << "1LaneLastId: " << laneLast->mId << std::endl;

        if ( roadLast->mId == roadInId )
        {
            return 1;
        }
        else
        {
            return 0;
        }

    }

    // get predecessor if existed
    else if ( roadNow->getSuccessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadLast = roadNow->getSuccessor();
        laneSecLast = roadLast->getFirstLaneSection();
        if ( laneNow->getSuccessor() )
        {
            laneLast = laneNow->getSuccessor();
        }
        else
        {
            return 0;
        }
        //if (laneLast == 0)
        //{
        //
        //}
        //else
        //{
        //    std::cerr << "2RoadLastId: " << roadLast->mId << std::endl;
        //    std::cerr << "2LaneLastId: " << laneLast->mId << std::endl;
        //}

        if ( roadLast->mId == roadInId )
        {
            return 1;
        }
        else
        {
            return 0;
        }

        
    }

    else
    {
        //std::cerr << "In 3" << std::endl;
        if (!roadNow->getSuccessorNode())
        {
            //std::cerr << "BOTW" << std::endl;
            return 0;
        }

        OpenDrive::JuncHeader* juncHdr = reinterpret_cast< OpenDrive::JuncHeader* >( roadNow->getSuccessorNode() );

        if ( !juncHdr->getFirstLink() )
        {
            return 0;
        }
        OpenDrive::JuncLink* jclink = juncHdr->getFirstLink();
        //std::cerr << "31" << std::endl;

        //std::cerr << "roadNow->mId:" << roadNow->mId << std::endl;
        //std::cerr << "roadInId:" << roadInId << std::endl;
        while ( (jclink->mConnectingRoad->getSuccessor() && jclink->mConnectingRoad->getSuccessor()->mId != roadNow->mId ) || (jclink->mConnectingRoad && jclink->mConnectingRoad->mId != roadInId) )
        {
            //std::cerr << "jclink->mConnectingRoad->getSuccessor()->mId:" << jclink->mConnectingRoad->getSuccessor()->mId << std::endl;
            //std::cerr << "jclink->mConnectingRoad->mId:" << jclink->mConnectingRoad->mId << std::endl;
            if ( jclink->getRight() )
            {
                jclink = reinterpret_cast< OpenDrive::JuncLink* >( jclink->getRight() );
            }
            else
            {
                return 0;
            }

        }


        //std::cerr << "roadfindout: " << jclink->mConnectingRoad->mId << std::endl;

        roadLast = jclink->mConnectingRoad;

        //std::cerr << "roadIn: " << roadInId << std::endl;
        //std::cerr << "roadfind: " << roadLast->mId << std::endl;

        laneSecLast = roadLast->getLastLaneSection();
        //std::cerr << "32" << std::endl;
        int cnt = 10;
        while (cnt > -10)
        {
            //std::cerr << "found lane suc"<< lsec->getLaneFromId(cnt)->getSuccessor()->mId << std::endl;
            if( laneSecLast->getLaneFromId(cnt) )
            {
                if ( laneSecLast->getLaneFromId(cnt)->getSuccessor() )
                {
                    //std::cerr << "good: "<< laneSecLast->getLaneFromId(cnt)->getSuccessor()->mId << std::endl;
                    if ( laneSecLast->getLaneFromId(cnt)->getSuccessor()->mId == laneNow->mId )
                    {
                        laneLast = laneSecLast->getLaneFromId(cnt);

                        //std::cerr << "3RoadLastId: " << roadLast->mId << std::endl;
                        //if (laneLast->mId)
                        //{
                        //    std::cerr << "3LaneLastId: " << laneLast->mId << std::endl;
                        //}

                        return 1;
                    }
                }

            }        
            --cnt;
        }
        //std::cerr << "3end" << std::endl;


        //std::cerr << "3RoadLastId: " << roadLast->mId << std::endl;
        //if (laneLast->mId)
        //{
        //    std::cerr << "3LaneLastId: " << laneLast->mId << std::endl;
        //}
        
        
    }


}


void 
RouteList::getNextLaneSec( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadNext, OpenDrive::LaneSection* &laneSecNext, OpenDrive::Lane* &laneNext, int roadDir)
{
    //If next lanesection exist, return it rather than next road. 
    if ( laneSecNow->getRight() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecNext = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );
        //std::cerr << "1" << std::endl;
        roadNext = roadNow;//*reinterpret_cast< OpenDrive::RoadHeader* >( roadNow->getCopy(true) );
        //std::cerr << "roadNext:" << roadNext->mId << std::endl;
        laneSecNext = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );
        //std::cerr << "2" << std::endl;

        //std::cerr << "laneNow->getSuccessor()->mId: " << laneNow->getSuccessor()->mId << std::endl;
        if ( laneNow->getSuccessor() )
        {
        	laneNext = laneNow->getSuccessor();   
        }
        //else
        //{
        //	while (laneNow->getRight() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getRight() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneNext = laneNow->getSuccessor();        		
        //		}
        //		if (laneNext)
        //		{
        //			return;
        //		}
        //	}
		//
        //	while (laneNow->getLeft() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getLeft() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneNext = laneNow->getSuccessor();        		
        //		}
        //		if (laneNext)
        //		{
        //			return;
        //		}
        //	}
        //}

    }
    // get successor if existed
    
    else if ( roadNow->getSuccessor() && laneNow->getSuccessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadNext = roadNow->getSuccessor();
        laneSecNext = roadNext->getFirstLaneSection();

        laneNext = laneNow->getSuccessor();
        if (laneNext->getFirstWidth() && laneNext->mId >= 0)
        {
            laneSecNext = roadNext->getLastLaneSection();
        }


        //if (roadNext->mId)
        //    std::cerr << "2RoadNextId: " << roadNext->mId << std::endl;
        //if (laneNext->mId)
        //    std::cerr << "2LaneNextId: " << laneNext->mId << std::endl;        
        
        
    }

    else
    {   
        //std::cerr << "In 3" << std::endl;
        getNextRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, roadDir);
    }


}

void 
RouteList::getReverseNextLaneSec( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadNext, OpenDrive::LaneSection* &laneSecNext, OpenDrive::Lane* &laneNext, int roadDir)
{
    //If next lanesection exist, return it rather than next road.
    if ( laneSecNow->getLeft() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecNext = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        //std::cerr << "1" << std::endl;
        roadNext = roadNow;//*reinterpret_cast< OpenDrive::RoadHeader* >( roadNow->getCopy(true) );
        laneSecNext = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        //std::cerr << "2" << std::endl;

        //std::cerr << "laneNow->getSuccessor()->mId: " << laneNow->getSuccessor()->mId << std::endl; 
        if ( laneNow->getPredecessor() )
        {
        	laneNext = laneNow->getPredecessor();   
        }
        //else
        //{
        //	while (laneNow->getRight() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getRight() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneNext = laneNow->getPredecessor();        		
        //		}
        //		if (laneNext)
        //		{
        //			return;
        //		}
        //	}
		//
        //	while (laneNow->getLeft() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getLeft() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneNext = laneNow->getPredecessor();        		
        //		}
        //		if (laneNext)
        //		{
        //			return;
        //		}
        //	}
        //}

    }
    // get successor if existed
    
    else if ( roadNow->getPredecessor() && laneNow->getPredecessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadNext = roadNow->getPredecessor();
        laneSecNext = roadNext->getLastLaneSection();

        laneNext = laneNow->getPredecessor();
        if (laneNext->getFirstWidth() && laneNext->mId < 0)
        {
            laneSecNext = roadNext->getFirstLaneSection();
        }
        //if (roadNext->mId)
        //    std::cerr << "2RoadNextId: " << roadNext->mId << std::endl;
        //if (laneNext->mId)
        //    std::cerr << "2LaneNextId: " << laneNext->mId << std::endl;        
        
        
    }

    else
    {   
        getLastRoad( roadNow, laneSecNow, laneNow, roadNext, laneSecNext, laneNext, roadDir);
    }

}

int 
RouteList::getLastLaneSec( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadLast, OpenDrive::LaneSection* &laneSecLast, OpenDrive::Lane* &laneLast )
{
    //If last lanesection exist, return it rather than last road.
    if ( laneSecNow->getLeft() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        roadLast = roadNow;
        laneSecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getLeft() );
        
        if ( laneNow->getPredecessor() )
        {
        	laneLast = laneNow->getPredecessor();   
        }
        //else
        //{
        //	while (laneNow->getRight() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getRight() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneLast = laneNow->getPredecessor();        		
        //		}
        //		if (laneLast)
        //		{
        //			return 0;
        //		}
        //	}
		//
        //	while (laneNow->getLeft() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getLeft() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneLast = laneNow->getPredecessor();        		
        //		}
        //		if (laneLast)
        //		{
        //			return 0;
        //		}
        //	}
        //}

        return 0;
    }

    // get predecessor if existed
    else if ( roadNow->getPredecessor() && laneNow->getPredecessor() )
    {
        roadLast = roadNow->getPredecessor();
        laneSecLast = roadLast->getLastLaneSection();

        laneLast = laneNow->getPredecessor();
        if (laneLast->getFirstWidth() && laneLast->mId < 0)
        {
            laneSecLast = roadLast->getFirstLaneSection();
        }

        //if (laneLast == 0)
        //{
        //
        //}
        //else
        //{
        //    std::cerr << "2RoadLastId: " << roadLast->mId << std::endl;
        //    std::cerr << "2LaneLastId: " << laneLast->mId << std::endl;
        //}
        return 0;
        
    }

    return 1;

}

int 
RouteList::getReverseLastLaneSec( OpenDrive::RoadHeader* roadNow, OpenDrive::LaneSection* laneSecNow, OpenDrive::Lane* laneNow, OpenDrive::RoadHeader* &roadLast, OpenDrive::LaneSection* &laneSecLast, OpenDrive::Lane* &laneLast )
{
    //If last lanesection exist, return it rather than last road.
    if ( laneSecNow->getRight() && laneNow->getSuccessor() )
    {
        //std::cerr << "In 1" << std::endl;
        //OpenDrive::LaneSection* lsecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );
        roadLast = roadNow;
        laneSecLast = reinterpret_cast< OpenDrive::LaneSection* >( laneSecNow->getRight() );

        if ( laneNow->getSuccessor() )
        {
        	laneLast = laneNow->getSuccessor();   
        	return 0;
        }
        //else
        //{
        //	while (laneNow->getRight() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getRight() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneLast = laneNow->getSuccessor();        		
        //		}
        //		if (laneLast)
        //		{
        //			return 0;
        //		}
        //	}
		//
        //	while (laneNow->getLeft() )
        //	{
        //		laneNow = reinterpret_cast< OpenDrive::Lane* >( laneNow->getLeft() );
        //		if (laneNow && ifDriveable(laneNow))
        //		{
        //			laneLast = laneNow->getSuccessor();        		
        //		}
        //		if (laneLast)
        //		{
        //			return 0;
        //		}
        //	}
        //}

        return 0;
    }

    // get predecessor if existed
    else if ( roadNow->getSuccessor() && laneNow->getSuccessor() )
    {
        //std::cerr << "In 2" << std::endl;
        roadLast = roadNow->getSuccessor();
        laneSecLast = roadLast->getFirstLaneSection();

        laneLast = laneNow->getSuccessor();
        if (laneLast->getFirstWidth() && laneLast->mId >= 0)
        {
            laneSecLast = roadLast->getLastLaneSection();
        }

        //if (laneLast == 0)
        //{
        //
        //}
        //else
        //{
        //    std::cerr << "2RoadLastId: " << roadLast->mId << std::endl;
        //    std::cerr << "2LaneLastId: " << laneLast->mId << std::endl;
        //}
        return 0;
        
    }

    return 1;

}

void
RouteList::trimNextLanes(std::list<OpenDrive::Lane*>& nextLanes, int tarId)
{
    int start = 0;
    int end = 0;
    int lastId = 1000;
    int found = 0;

    for (std::list<OpenDrive::Lane*>::iterator i = nextLanes.begin(); i != nextLanes.end(); ++i, ++end)
    {
        if (lastId != 1000)
        {
            //if not continuous
            if( abs((*i)->mId - lastId) != 1 )
            {
                //if not found, this part is useless
                if (!found)
                {
                    start = end;
                }
                else // this part is what we need. end loop
                {
                    //++end;
                    break;
                }
            }
        }

        if ((*i)->mId == tarId)
        {
            found = 1;
        }

        lastId = (*i)->mId;
    }

    end = nextLanes.size() - end;

    // std::cerr << "tarId: "<< tarId << std::endl;
    // std::cerr << "start: "<< start << std::endl;
    // std::cerr << "end: "<< end << std::endl;



    for(;start;--start)
    {
        nextLanes.pop_front();
    }

    for(;end;--end)
    {
        nextLanes.pop_back();
    }

}


}// namespace Module