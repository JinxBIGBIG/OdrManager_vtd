/* ===================================================
 *  file:       OdrLaneSpeed.hh
 * ---------------------------------------------------
 *  purpose:	LaneSpeed information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_SPEED_HH
#define _OPENDRIVE_LANE_SPEED_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneSpeed : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mOffset;
        double mSpeed;

        // new in ODR 1.4
        unsigned int  mUnit;

    public:
        /**
        * constructor
        */
        explicit LaneSpeed();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneSpeed( LaneSpeed *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneSpeed();
        
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
        * calculate speed from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return speed at this position
        */
        double ds2speed( const double & ds );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_SPEED_HH */
