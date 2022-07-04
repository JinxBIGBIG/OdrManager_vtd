/* ===================================================
 *  file:       OdrRoadSpeed.hh
 * ---------------------------------------------------
 *  purpose:	RoadSpeed information
 * ---------------------------------------------------
 *  first edit:	03.03.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_SPEED_HH
#define _OPENDRIVE_ROAD_SPEED_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadSpeed : public Node
{
    public:
        /** conversion factors
        */
        static const double scMph2ms;    // miles per hour to meters per second
        static const double scKmh2ms;    // kilometers per hour meters per second
    
    public:
        /**
        * member variables according to specification
        */
        double       mMax;
        unsigned int mUnit;

    public:
        /**
        * constructor
        */
        explicit RoadSpeed();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadSpeed( RoadSpeed *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadSpeed();
        
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
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ROAD_SPEED_HH */
