/* ===================================================
 *  file:       OdrParkingSpaceMarking.hh
 * ---------------------------------------------------
 *  purpose:	Parking Space Marking information
 * ---------------------------------------------------
 *  first edit:	14.04.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_PARKING_SPACE_MARKING_HH
#define _OPENDRIVE_PARKING_SPACE_MARKING_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class ParkingSpaceMarking : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mSide;
        unsigned int mType;
        double       mWidth;
        unsigned int mColor;

    public:
        /**
        * constructor
        */
        explicit ParkingSpaceMarking();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit ParkingSpaceMarking( ParkingSpaceMarking *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~ParkingSpaceMarking();
        
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

#endif /* _OPENDRIVE_PARKING_SPACE_MARKING_HH */
