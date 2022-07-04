/* ===================================================
 *  file:       OdrTrafficObject.hh
 * ---------------------------------------------------
 *  purpose:	traffic object to be handled
 *              by LaneObserver
 * ---------------------------------------------------
 *  first edit:	27.08.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_TRAFFIC_OBJECT_HH
#define _OPENDRIVE_TRAFFIC_OBJECT_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

// forward declarations
class RoadHeader;
class LaneSection;
class Lane;

class TrafficObject : public Node
{
    public:
        /**
        * s position within the respective lane
        */
        double mS;

        /**
        * id of the traffic object
        */
        int mId;

    public:
        /**
        * constructor
        */
        explicit TrafficObject();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit TrafficObject( TrafficObject *rhs );
        
        /**
        * constructor
        * @param s  s co-ordinate of the object
        * @param id unique ID of the traffic object
        */
        explicit TrafficObject( const double & s, const int & id );

        /** 
        * Destroy the class. 
        */
        virtual ~TrafficObject();
        
        /**
        * print the data contents
        */
        virtual void printData() const;
        
        /**
        * get hierarchy information
        * @return the road header under which the object is registered
        */
        RoadHeader *getRoadHeader();
        
        /**
        * get hierarchy information
        * @return the lane section under which the object is registered
        */
        LaneSection *getLaneSection();
        
        /**
        * get hierarchy information
        * @return the lane under which the object is registered
        */
        Lane *getLane();
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_TRAFFIC_OBJECT_HH */
