/* ===================================================
 *  file:       OdrGeoReference.hh
 * ---------------------------------------------------
 *  purpose:	GeoReference information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_GEO_REFERENCE_HH
#define _OPENDRIVE_GEO_REFERENCE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    class RoadData;

class GeoReference : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string mDataString;

    public:
        /**
        * constructor
        */
        explicit GeoReference();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit GeoReference( GeoReference *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~GeoReference();
        
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

#endif /* _OPENDRIVE_ROAD_TYPE_HH */
