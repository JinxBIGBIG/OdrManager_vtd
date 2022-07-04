/* ===================================================
 *  file:       OdrLaneSection.hh
 * ---------------------------------------------------
 *  purpose:	LaneSection information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_SECTION_HH
#define _OPENDRIVE_LANE_SECTION_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

//forward declaration
class Lane;
    
class LaneSection : public Node
{
    public:
        static const unsigned char sLaneSecSideFlagNone   = 0x00;
        static const unsigned char sLaneSecSideFlagLeft   = 0x01;
        static const unsigned char sLaneSecSideFlagRight  = 0x02;
        static const unsigned char sLaneSecSideFlagCenter = 0x04;
        static const unsigned char sLaneSecSideFlagAll    = 0x07;
        
        /**
        * member variables according to specification
        */
        double mS;
        
        /**
        * additional variables for higher performance
        */
        double mSEnd;
        
        /**
        * validity of the lane section for a given side
        */
        unsigned char mSideMask;
        
    private:
        /**
        * maximum width of a side within this section
        */
        double mMaxHalfWidth;

    public:
        /**
        * constructor
        */
        explicit LaneSection();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneSection( LaneSection *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneSection();
        
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
        * get a specific lane in this section
        * @param id lane id
        * @return pointer to the lane or 0
        */
        Lane* getLaneFromId ( int id );
        
        /**
        * add border lanes to lane section
        * @param widthLeft   width of left border lane
        * @param typeLeft    type of left border lane
        * @param wRight      width of right border lane
        * @param typeRight   type of right border lane
        */
        void addBorderLanes( const double & widthLeft, const unsigned int & typeLeft,
                             const double & widthRight, const unsigned int & typeRight );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

        /**
        * calculate the respective data in the preparation stage 
        */
        virtual void calcPrepareData();

        /**
        * calculate the respective data after the preparation stage 
        */
        virtual void calcPostPrepareData();
        
        /**
        * get the maximum width on either side of the chord line
        * @return maximum width of either side
        */
        const double & getMaxHalfWidth();
        
        /**
        * sort the lanes within the lane section
        */
        virtual void sortLanes();
        
        /**
        * check whether a lane section is valid for a given side of the road
        * @param laneId id of the lane which is to be checked vs. the side mask of the lane section
        * @return true if lane with given id is potentially part of the lane section (actual existence of the lane is not checked or confirmed)
        */
        bool containsSide( const int & laneId );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LANE_SECTION_HH */
