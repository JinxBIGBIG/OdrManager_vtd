/* ===================================================
 *  file:       OdrRoadMarkSway.hh
 * ---------------------------------------------------
 *  purpose:	Roadmark Sway information
 * ---------------------------------------------------
 *  first edit:	31.05.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_MARK_SWAY_HH
#define _OPENDRIVE_ROAD_MARK_SWAY_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadMarkSway : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mDs;
        double mA;
        double mB;
        double mC;
        double mD;
        
        /**
        * derived variables
        */
        double mS;
        double mSEnd;

    public:
        /**
        * constructor
        */
        explicit RoadMarkSway();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadMarkSway( RoadMarkSway *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadMarkSway();
        
        /**
        * print the data contents
        */
        virtual void printData() const;
        
       /**
        * read the contents of the node from a prepared attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        bool read( ReaderXML* reader );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

        /**
        * get the sway offset at a given s position
        * @param s  track co-ordinate
        * @return sway in [m]
        */
        double s2offset( const double & s );
        
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();
        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ROAD_MARK_SWAY_HH */
