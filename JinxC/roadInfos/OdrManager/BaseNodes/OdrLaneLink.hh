/* ===================================================
 *  file:       OdrLaneLink.hh
 * ---------------------------------------------------
 *  purpose:	LaneLink information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_LINK_HH
#define _OPENDRIVE_LANE_LINK_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneLink : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned short mType;
        int mLaneId;

    public:
        /**
        * constructor
        */
        explicit LaneLink();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneLink( LaneLink *rhs );

        /**
        * constructor
        * @param type define the type of the link
        */
        explicit LaneLink( unsigned short type );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneLink();
        
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

#endif /* _OPENDRIVE_LANE_LINK_HH */
