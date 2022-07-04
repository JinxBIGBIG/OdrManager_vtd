/* ===================================================
 *  file:       OdrPositionRoad.hh
 * ---------------------------------------------------
 *  purpose:	PositionRoad information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_POSITION_ROAD_HH
#define _OPENDRIVE_POSITION_ROAD_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class PositionRoad : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string mRoadId;
        double      mS;
        double      mT;
        double      mOffsetZ;
        double      mOffsetH;
        double      mPitch;
        double      mRoll;       

    public:
        /**
        * constructor
        */
        explicit PositionRoad();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit PositionRoad( PositionRoad *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~PositionRoad();
        
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

#endif /* _OPENDRIVE_POSITION_ROAD_HH */
