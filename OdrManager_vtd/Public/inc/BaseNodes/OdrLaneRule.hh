/* ===================================================
 *  file:       OdrLaneRule.hh
 * ---------------------------------------------------
 *  purpose:	LaneRule information
 * ---------------------------------------------------
 *  first edit:	10.08.2017 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_RULE_HH
#define _OPENDRIVE_LANE_RULE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneRule : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mOffset;
        std::string mValue;
        
        /**
        * further variables for faster navigation and evaluation
        */
        double mS;      // absolute s co-ordinate at begin of entry  [m]
        double mSEnd;   // aboslute s co-ordinate at end of entry    [m]

    public:
        /**
        * constructor
        */
        explicit LaneRule();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneRule( LaneRule *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneRule();
        
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
        * get rule from a given ds to lane section entry
        * and dt from inner border of lane
        * @param ds delta track-pos relative to lane section entry
        * @return rule at this position or empty string
        */
        std::string ds2rule( const double & ds );
        
        /**
        * get rule from a given s (track co-ordinate)
        * and relative offset (normalized with actual width) from inner border
        * @param s  track-pos relative to begin of track
        * @return rule at this position or empty string
        */
        std::string s2rule( const double & s );
        
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

#endif /* _OPENDRIVE_LANE_RULE_HH */
