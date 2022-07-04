/* ===================================================
 *  file:       OdrDataQuality.hh
 * ---------------------------------------------------
 *  purpose:	ancillary data quality data
 * ---------------------------------------------------
 *  first edit:	03.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_DATA_QUALITY_HH
#define _OPENDRIVE_DATA_QUALITY_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class DataQuality : public Node
{
    public:
        /**
        * constructor
        */
        explicit DataQuality();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit DataQuality( DataQuality *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~DataQuality();
        
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

#endif /* _OPENDRIVE_DATA_QUALITY_HH */
