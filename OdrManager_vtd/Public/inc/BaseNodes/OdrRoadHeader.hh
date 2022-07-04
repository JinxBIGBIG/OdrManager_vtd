/* ===================================================
 *  file:       OdrRoadHeader.hh
 * ---------------------------------------------------
 *  purpose:	road header information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_HEADER_HH
#define _OPENDRIVE_ROAD_HEADER_HH

// ====== INCLUSIONS ======
#include <vector>
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declarations
class GeoHeader;
class Elevation;
class LateralShape;
class Crossfall;
class Superelevation;
class LaneSection;
class RoadLink;
class Signal;
class Lane;
class Bbox;
class RoadType;
class SurfaceCRG;
class Tunnel;
class Bridge;
class Object;
class LaneOffset;

class RoadHeader : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mName;
        double       mLength;
        unsigned int mId;           // the numeric ID derived from the original ID
        std::string  mIdAsString;   // the original ID as given in the OpenDRIVE file
        int          mJuncNo;
        std::string  mJuncNoAsString;
        bool         mRHT;          // right hand traffic
        
        /**
        * additional member variables which are pre-computed
        */
        double  mMaxHalfWidth;      // maximum width to one side of road

        /**
        * tesselation length for continuous objects
        */
        float mContObjTesselation;
        
        /**
        * resolve repeated objects?
        */
        bool mResolveRepeatedObjects;


    public:
        /**
        * constructor
        */
        explicit RoadHeader();
        
        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadHeader( RoadHeader *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadHeader();
        
        /**
        * print the data contents
        */
        virtual void printData() const;

        double getLength() const;

        unsigned int getId() const;

        std::string getIdStr() const;

        int getJunctionNo() const;
        
        /**
        * read the contents of the node from a prepared attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        bool read( ReaderXML* reader );

        /**
        * member access method
        * @return the first geometry header
        */
        GeoHeader* getFirstGeoHeader();

        /**
        * member access method
        * @return the last geometry header
        */
        GeoHeader* getLastGeoHeader();
        
        /**
        * member access method
        * @return the first elevation entry
        */
        Elevation* getFirstElevation();
        
        /**
        * member access method
        * @return the first lateral shape entry
        */
        LateralShape* getFirstLateralShape();
        
        /**
        * member access method
        * @return the first crossfall entry
        */
        Crossfall* getFirstCrossfall();
        
        /**
        * member access method
        * @return the first superelevation entry
        */
        Superelevation* getFirstSuperelevation();
        
        /**
        * member access method
        * @return the first lane section node
        */
        LaneSection* getFirstLaneSection();

        /**
        * member access method
        * @return the last lane section node
        */
        LaneSection* getLastLaneSection();

        /**
        * member access method
        * @return the first lane section node
        */
        LaneOffset* getFirstLaneOffset();

        /**
        * member access method
        * @return the last lane section node
        */
        LaneOffset* getLastLaneOffset();

        /**
        * member access method
        * @return the first signal entry
        */
        Signal* getFirstSignal();
        
        /**
        * member access method
        * @return the first tunnel entry
        */
        Tunnel* getFirstTunnel();
        
        /**
        * member access method
        * @return the first bridge entry
        */
        Bridge* getFirstBridge();
        
        /**
        * member access method
        * @return the first object entry
        */
        Object* getFirstObject();
        
        /**
        * member access method
        * @return the first road type entry
        */
        RoadType* getFirstRoadType();

        /**
        * member access method
        * @return the first link information
        */
        RoadLink* getFirstLink();
        
        /**
        * member access method
        * @param type the type of the linking element that shall be returned
        * @return the predecessor link information
        */
        RoadLink* getPredecessorLink( unsigned short type = ODR_TYPE_ROAD );
        
        /**
        * member access method
        * @param type the type of the linking element that shall be returned
        * @return the successor link information
        */
        RoadLink* getSuccessorLink( unsigned short type = ODR_TYPE_ROAD );
        
        /**
        * member access method
        * @param followJunction set to true in order to follow automatically a clear junction connection
        * @return header of the preceding road or 0
        */
        RoadHeader* getPredecessor( bool followJunction = false );
        
        /**
        * member access method
        * @param followJunction set to true in order to follow automatically a clear junction connection
        * @return header of the succeeding road or 0
        */
        RoadHeader* getSuccessor( bool followJunction = false );
        
        /**
        * member access method
        * @return preceding node (road or junction)
        */
        Node* getPredecessorNode();
        
        /**
        * member access method
        * @return succeeding node (road or junction)
        */
        Node* getSuccessorNode();
        
        /**
        * get the link direction of the preceding road
        * @return link direction
        */
        const unsigned char & getPredecessorDir() const;
        
        /**
        * get the link direction of the succceding road
        * @return link direction
        */
        const unsigned char & getSuccessorDir() const;
        
        /**
        * get the position where the predecessor starts
        * (relevant for railroad switches)
        * @return position of link on predecessor
        */
        double getPredecessorS() const;
        
        /**
        * get the position where the successor starts
        * (relevant for railroad switches)
        * @return position of link on successor
        */
        double getSuccessorS() const;
        
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();

        /**
        * calculate the respective data after the preparation stage 
        */
        virtual void calcPostPrepareData();
        
        /**
        * register a road as neighbor to another road
        * @param neighbor the road that shall be registered as neighbor
        * @version 1.5.3
        */
        void registerNeighbor( RoadHeader* neighbor );
        
        /**
        * get the neighbor at the given index
        * @param index index of the requested neighbor
        */
        RoadHeader* getNeighbor( unsigned int index );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * invalidate all lanes except for the indicated one
        * @param remain         id of the remaining lane
        * @param drviveableOnly set to true in order to eliminate driveable lanes only
        */
        void invalidateLanes( int remain = 0, bool driveableOnly = false );
        
        /**
        * get the pointer to a lane in the very first
        * lane section of the Road
        * @param no number of the lane
        * @return pointer to the lane or 0
        */
        Lane* getLaneInFirstSection( int no );
        
        /**
        * get the pointer to the junction
        * @return pointer to junction node or 0
        */
        Node* getJunction();
        
        /**
        * set the pointer to the junction
        * @param junc pointer to junction node
        */
        void setJunction( Node* junc );
        
        /**
        * get the maximum number of driveable lanes in
        * any given road section
        * @return number of driveable lanes
        */
        unsigned int getNoDriveableLanes();

        /**
        * check the bounding box of the element contains a given point
        * @param x  x-position
        * @param y  y-position
        * @return   true if position is in bounding box
        */
        bool inBoundingBox( const double & x, const double & y );
        
        /**
        * get the bounding box
        */
        Bbox* getBoundingBox();
        
        /**
        * calculate the bounding box
        * @param halfWidth  if non-zero, use the given road half width instead of
        *                   an automatically computed one 
        */
        void calcBoundingBox( const double & halfWidth = 0.0 );
        
        /**
        * check whether this road has surface data
        * @param purpose    the purpose (mask)for which the surface data may be used
        * @return true if surface data is available, otherwise false
        */
        bool hasSurfaceData( const unsigned short & purpose = 0xffff );
        
        /**
        * convert a road position into the corresponding SurfaceCRG node
        * @param s          s position for which to search geo header
        * @param purpose    the purpose (mask) for which the surface data may be used
        * @return surfaceCRG node matching the track position (ID, s)
        */
        SurfaceCRG* s2surface( const double & s, const unsigned short & purpose = 0xffff  );

        /**
        * get the maximum absolute curvature in a given range of the reference line
        * @param start   minimum s position
        * @param stop   maximum s position
        * @return maximum curvature (max in terms of abs value) 
        */
        double getMaxCurvatureInRange( const double & start, const double & stop );
        
        /**
        * set numeric ID and, optionally, convert numeric ID to string ID
        * @param    id numeric ID
        * @param    convert if true, set also string ID
        */
        void setId( const unsigned int & id, bool convert = true );
        
        /**
        * set junction ID and, optionally, convert numeric ID to string ID
        * @param    id numeric ID
        * @param    convert if true, set also string ID
        */
        void setJuncNo( const int & id, bool convert = true );
        
        /**
        * print the neighbor nodes
        * @version 1.5.3
        */
        void printNeighbors();
        
    private:
        /**
        * keep info about predecessor and successor direction
        * once it is computed
        */
        unsigned char mPredElemDir;
        unsigned char mSuccElemDir;
        unsigned char mJuncPredElemDir;
        unsigned char mJuncSuccElemDir;
        
        /**
        * keep info about position of link on successor / predecessor
        */
        double mPredS;
        double mSuccS;
        double mJuncPredS;
        double mJuncSuccS;
        
        /**
        * keep pointers to predecessor and successor node (if it's a road)
        */
        Node *mPredecessor;
        Node *mSuccessor;
        RoadHeader *mJuncSuccessor;       // successor in junction if connection is clear
        RoadHeader *mJuncPredecessor;     // predecessor in junction if connection is clear
        Node *mJuncHeader;
        
        /**
        * hold a vector of potential neighbors other than immediate
        * predecessor and successor (for faster evaluation of junctions)
        */
        typedef std::vector<RoadHeader*> NeighborVec;
        NeighborVec mNeighbors;
        
        /**
        * bounding box of road
        */
        Bbox *mBbox;
        
        /** 
        * additional surface data available?
        */
        bool mHasSurfaceData;
       
    private:
        
        /**
        * calculate links for railroad switches
        */
        void calcRRLinks();
        
        /**
        * calculate the predecessor (road or junction)
        */
        void calcPredecessor();
        
        /**
        * calculate the successor (road or junction)
        */
        void calcSuccessor();
    
        /**
        * get the road's neighbor from the provided link information
        * @param link road link information
        * @return neighbor or 0
        */
        Node* getNeighborFromLink( RoadLink* link );
        
        /**
        * sort child entries (e.g. signals) for increasing s position
        */
        void sortChildren();
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ROAD_HEADER_HH */
