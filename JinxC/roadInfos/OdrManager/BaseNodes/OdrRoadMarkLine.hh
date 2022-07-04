/* ===================================================
 *  file:       OdrRoadMarkLine.hh
 * ---------------------------------------------------
 *  purpose:	road mark information
 * ---------------------------------------------------
 *  first edit:	06.03.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_MARK_LINE_HH
#define _OPENDRIVE_ROAD_MARK_LINE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadMarkLine : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mLength;
        double         mSpace;
        double         mOffsetT;
        double         mOffsetS;
        unsigned short mRule;
        double         mWidth;
        bool           mHasWidth;
        std::string    mColor;  /** @version 1.5.0 */
        bool           mHasColor;  /** @version 1.5.0 */

    public:
        /**
        * constructor
        */
        explicit RoadMarkLine();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadMarkLine( RoadMarkLine *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadMarkLine();
        
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

#endif /* _OPENDRIVE_ROAD_MARK_LINE_HH */
