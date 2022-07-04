/* ===================================================
 *  file:       OdrLane.hh
 * ---------------------------------------------------
 *  purpose:	Lane information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_HH
#define _OPENDRIVE_LANE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

// forward declarations
class LaneWidth;
class RoadMark;
class LaneLink;
class LaneBorder;

class Lane : public Node
{
    public:
        /**
        * member variables according to specification
        */
        int          mId;
        unsigned int mType;
        unsigned int mSurfLevel;

    public:
        /**
        * constructor
        */
        explicit Lane();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Lane( Lane *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Lane();
        
        /**
        * print the data contents
        */
        virtual void printData() const;

        int getId() const;
        unsigned int getType() const;
        unsigned int getSurfLevel() const;
        
        /**
        * read the contents of the node from a prepared attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        bool read( ReaderXML* reader );
        
        /**
        * member access method
        * @return the first width node
        */
        LaneWidth* getFirstWidth();
        
        /**
        * member access method
        * @return the first border node
        */
        LaneBorder* getFirstBorder();
        
        /**
        * member access method
        * @return the first road mark node
        */
        RoadMark* getFirstRoadMark();
        
        /**
        * member access method
        * @return the first lnk node
        */
        LaneLink* getFirstLink();
        
        /**
        * get the link node for the lane's predecessor
        * @return laneLink of predecessor or NULL
        */
        LaneLink* getPredecessorLink();
        
        /**
        * get the link node for the lane's successor
        * @return laneLink of successor or NULL
        */
        LaneLink* getSuccessorLink();
        
        /**
        * insert a child in ascending s order
        * @param child child node
        * @return true upon sucess
        */
        bool insertChildByS( Node* child );
                
        /**
        * find the logical predecessor of the lane
        * @return predecessor of the lane or 0 if none exists
        */
        Lane* getPredecessor();
        
        /**
        * find the logical successor of the lane
        * @return successor of the lane or 0 if none exists
        */
        Lane* getSuccessor();
        
        /**
        * get the link direction of the preceding lane
        * @return link direction
        */
        const unsigned char & getPredecessorDir() const;
        
        /**
        * get the link direction of the succceding lane
        * @return link direction
        */
        const unsigned char & getSuccessorDir() const;
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * check whether a lane is driveable for cars / railroad
        * @param allowCar if true, driving lanes are allowed
        * @param allowRail if true, railroad lanes are allowed
        * @return true if lane is driveable
        */
        bool isDriveable( bool allowCar = true, bool allowRail = false );
         
        /**
        * get the maximum and minimum value within the definition range
        * @param minWidth reference to minimum width within range (to be altered)
        * @param maxWidth reference to maximum width within range (to be altered)
        */
        void getWidthMinMax( double & minWidth, double & maxWidth );

    private:
        /**
        * keep info about predecessor and successor direction
        * once it is computed
        */
        unsigned char mPredElemDir;
        unsigned char mSuccElemDir;

        /**
        * keep pointers to predecessor and successor
        */
        Lane *mPredecessor;
        Lane *mSuccessor;

    private:
        /**
        * find the logical predecessor of the lane
        * @return predecessor of the lane or 0 if none exists
        */
        Lane* calcPredecessor();
        
        /**
        * find the logical successor of the lane
        * @return successor of the lane or 0 if none exists
        */
        Lane* calcSuccessor();
        
        /**
        * get the link information for a certain type of link
        * @param type link type
        * @return the corresponding link or 0
        */
        LaneLink* getLinkFromType( unsigned short type );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_HH */
