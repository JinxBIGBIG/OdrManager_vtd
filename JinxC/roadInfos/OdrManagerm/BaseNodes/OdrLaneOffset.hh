/* ===================================================
 *  file:       OdrLaneOffset.hh
 * ---------------------------------------------------
 *  purpose:	offset of center line valid for all
 *              lanes
 * ---------------------------------------------------
 *  first edit:	20.07.2009 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_OFFSET_HH
#define _OPENDRIVE_LANE_OFFSET_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneOffset : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mS;
        double         mA;
        double         mB;
        double         mC;
        double         mD;

        /**
        * additional variables for higher performance
        */
        double mSEnd;
        
    public:
        /**
        * constructor
        */
        explicit LaneOffset();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneOffset( LaneOffset *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneOffset();
        
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
        * calculate offset from a given s
        * @param s track-pos
        * @return width at this position
        */
        double s2offset( const double & s );
        
        /**
        * calculate offsetDdot from a given s
        * @param s track-pos
        * @return offsetDot at this position
        */
        double s2offsetDot( const double & s );
        
        /**
        * calculate maximum value of offset within given range
        * @return maximum absolute offset within definition
        */
        double getMaxValue();
                
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
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

        /**
        * calculate curvature from a given s
        * @param s track-pos
        * @return curvature at this position
        */
        double s2curvature( const double & s );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_OFFSET_HH */
