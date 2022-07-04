/* ===================================================
 *  file:       OdrJuncLaneLink.hh
 * ---------------------------------------------------
 *  purpose:	JuncLaneLink information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_LANE_LINK_HH
#define _OPENDRIVE_JUNC_LANE_LINK_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
class Lane;

class JuncLaneLink : public Node
{
    public:
        /**
        * member variables according to specification
        */
        int mRoadLane;
        int mPathLane;
        
        /**
        * additional variables for higher performance
        */
        Lane* mIncomingLane;       // incoming lane
        Lane* mConnectingLane;     // connecting lane

    public:
        /**
        * constructor
        */
        explicit JuncLaneLink();

        /** 
        * Destroy the class. 
        */
        virtual ~JuncLaneLink();
        
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
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_LANE_LINK_HH */
