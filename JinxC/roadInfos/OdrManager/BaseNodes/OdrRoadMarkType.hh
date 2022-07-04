/* ===================================================
 *  file:       OdrRoadMarkType.hh
 * ---------------------------------------------------
 *  purpose:	road mark information
 * ---------------------------------------------------
 *  first edit:	06.03.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_MARK_TYPE_HH
#define _OPENDRIVE_ROAD_MARK_TYPE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadMarkType : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string mName;
        double      mWidth;
        bool        mHasWidth;

    public:
        /**
        * constructor
        */
        explicit RoadMarkType();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadMarkType( RoadMarkType *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadMarkType();
        
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
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_ROAD_MARK_TYPE_HH */
