/* ===================================================
 *  file:       OdrJuncLink.hh
 * ---------------------------------------------------
 *  purpose:	JuncLink information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_LINK_HH
#define _OPENDRIVE_JUNC_LINK_HH

// ====== INCLUSIONS ======

namespace OpenDrive
{
    
// forward declaration
class RoadHeader;
class JuncLaneLink;

class JuncLink : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int  mId;
        std::string   mIdAsString;
        unsigned int  mRoadId;
        std::string   mRoadIdAsString;
        unsigned int  mPathId;
        std::string   mPathIdAsString;
        unsigned char mDir;
        bool          mIsVirtual;
        std::string   mConnectionMasterId; 
        
        /**
        * additional variables for higher performance
        */
        RoadHeader* mIncomingRoad;       // incoming road
        RoadHeader* mConnectingRoad;     // connecting road
        JuncLink*   mConnectionMaster;
      
        /**
        * turn angle (relative change of direction) when following the path [-PI;PI] 
        * right turn is negative
        * left turn is positive
        * thresholds e.g. for indication have to be defined bz the user
        */
        double mTurnAngle;

    public:
        /**
        * constructor
        */
        explicit JuncLink();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit JuncLink( JuncLink *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~JuncLink();
        
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
        
        /**
        * get the pointer to the first lane link in the link
        * @return   pointer to first lane link
        */
        JuncLaneLink* getFirstLaneLink();
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_LINK_HH */
