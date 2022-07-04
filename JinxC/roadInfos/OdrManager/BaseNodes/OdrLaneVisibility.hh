/* ===================================================
 *  file:       OdrLaneVisibility.hh
 * ---------------------------------------------------
 *  purpose:	LaneVisibility information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_VISIBILITY_HH
#define _OPENDRIVE_LANE_VISIBILITY_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneVisibility : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mOffset;
        double mFwd;
        double mBack;
        double mLeft;
        double mRight;

    public:
        /**
        * constructor
        */
        explicit LaneVisibility();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneVisibility( LaneVisibility *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneVisibility();
        
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

#endif /* _OPENDRIVE_LANE_VISIBILITY_HH */
