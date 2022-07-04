/* ===================================================
 *  file:       OdrErrorDesc.hh
 * ---------------------------------------------------
 *  purpose:	ancillary error descriptor data
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ERROR_DESC_HH
#define _OPENDRIVE_ERROR_DESC_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class ErrorDesc : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mXYAbsolute;
        double mZAbsolute;
        double mXYRelative;
        double mZRelative;

    public:
        /**
        * constructor
        */
        explicit ErrorDesc();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit ErrorDesc( ErrorDesc *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~ErrorDesc();
        
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

#endif /* _OPENDRIVE_ERROR_DESC_HH */
