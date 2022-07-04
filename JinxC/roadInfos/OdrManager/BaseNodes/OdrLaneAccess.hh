/* ===================================================
 *  file:       OdrLaneAccess.hh
 * ---------------------------------------------------
 *  purpose:	LaneAccess information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_ACCESS_HH
#define _OPENDRIVE_LANE_ACCESS_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneAccess : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double       mOffset;
        unsigned int mRestriction;
        unsigned int mRule;

    public:
        /**
        * constructor
        */
        explicit LaneAccess();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneAccess( LaneAccess *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneAccess();
        
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
        * calculate restriction from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return restriction at this position
        */
        unsigned int ds2Restriction( const double & ds );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_ACCESS_HH */
