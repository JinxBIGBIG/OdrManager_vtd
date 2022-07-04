/* ===================================================
 *  file:       OdrRoadMark.hh
 * ---------------------------------------------------
 *  purpose:	road mark information
 * ---------------------------------------------------
 *  first edit:	26.05.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_MARK_HH
#define _OPENDRIVE_ROAD_MARK_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadMark : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mOffset;
        unsigned short mType;
        unsigned short mWeight;
        unsigned short mColor;
        double         mWidth;
        std::string    mMaterial;
        double         mHeight;
        
        /**
        * derived variables
        */
        double mS;
        double mSEnd;

    public:
        // added by JM -- 20211102
        double getOffset() const;
        unsigned short getType() const;
        unsigned short getWeight() const;
        unsigned short getColor() const;
        double getWidth() const;
        

        /**
        * constructor
        */
        explicit RoadMark();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadMark( RoadMark *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadMark();
        
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
        
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ROAD_MARK_HH */
