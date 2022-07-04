/* ===================================================
 *  file:       OdrRoadMarkExplicitLine.hh
 * ---------------------------------------------------
 *  purpose:	road mark information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_MARK_EXPLICIT_LINE_HH
#define _OPENDRIVE_ROAD_MARK_EXPLICIT_LINE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadMarkExplicitLine : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mLength;
        double         mOffsetT;
        double         mOffsetS;
        unsigned short mRule;
        double         mWidth;

    public:
        /**
        * constructor
        */
        explicit RoadMarkExplicitLine();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadMarkExplicitLine( RoadMarkExplicitLine *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadMarkExplicitLine();
        
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

#endif /* _OPENDRIVE_ROAD_MARK_EXPLICIT_LINE_HH */
