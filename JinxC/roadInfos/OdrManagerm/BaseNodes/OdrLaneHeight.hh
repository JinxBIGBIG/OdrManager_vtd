/* ===================================================
 *  file:       OdrLaneHeight.hh
 * ---------------------------------------------------
 *  purpose:	LaneHeight information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_HEIGHT_HH
#define _OPENDRIVE_LANE_HEIGHT_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneHeight : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mOffset;
        double mInner;
        double mOuter;
        
        /**
        * further variables for faster navigation and evaluation
        */
        double mS;      // absolute s co-ordinate at begin of lane height entry  [m]
        double mSEnd;   // aboslute s co-ordinate at end of lane height entry    [m]

    public:
        /**
        * constructor
        */
        explicit LaneHeight();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneHeight( LaneHeight *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneHeight();
        
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
        * calculate height from a given ds to lane section entry
        * and dt from inner border of lane
        * @param ds delta track-pos relative to lane section entry
        * @param dt delta lateral position relative to inner lane border
        * @return height at this position
        */
        double dsdt2height( const double & ds, const double & dt );
        
        /**
        * calculate height from a given s (track co-ordinate)
        * and relative offset (normalized with actual width) from inner border
        * @param s  track-pos relative to begin of track
        * @param tRel relative lateral offset from inner lane border
        * @return height at this position
        */
        double sAndRelOffset2height( const double & s, const double & tRel );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
         
        /**
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_HEIGHT_HH */
