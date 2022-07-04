/* ===================================================
 *  file:       OdrLaneBorder.hh
 * ---------------------------------------------------
 *  purpose:	LaneWidth information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_BORDER_HH
#define _OPENDRIVE_LANE_BORDER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
class Lane;

class LaneBorder : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mOffset;
        double mA;
        double mB;
        double mC;
        double mD;

        /**
        * additional variables for higher performance
        */
        double mOffsetEnd;
        
    private:
        /**
        * remember minimum and maximum width
        */
        double mMinWidth;
        double mMaxWidth;
        
        /**
        * remember lane which defines the inner border
        */
        Lane *mInnerLane;
        
        /**
        * remember inner lane's closest border entry
        */
        LaneBorder *mInnerLaneBorder;

    public:
        /**
        * constructor
        */
        explicit LaneBorder();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneBorder( LaneBorder *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneBorder();
        
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
        * calculate border offset from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return width at this position
        */
        double ds2borderT( const double & ds );
        
        /**
        * calculate first derivative of the border offset from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return width at this position
        */
        double ds2borderTDot( const double & ds );
        
        /**
        * calculate width from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return width at this position
        */
        double ds2width( const double & ds );
        
        /**
        * calculate first derivative of width from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return width at this position
        */
        double ds2widthDot( const double & ds );
        
        /**
        * calculate curvature from a given ds to lane section entry
        * @param ds delta track-pos to lane section entry
        * @return curvature at this position
        */
        double ds2Curv( const double & ds );
                
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
        * get the maximum and minimum value within the definition range
        * @param minWidth reference to minimum width within range (to be altered)
        * @param maxWidth reference to maximum width within range (to be altered)
        */
        void getMinMax( double & minWidth, double & maxWidth );
        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_BORDER_HH */
