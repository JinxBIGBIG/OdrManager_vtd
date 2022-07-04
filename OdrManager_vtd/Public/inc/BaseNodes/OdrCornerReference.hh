/* ===================================================
 *  file:       OdrCornerReference.hh
 * ---------------------------------------------------
 *  purpose:	CornerReference information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CORNER_REFERENCE_HH
#define _OPENDRIVE_CORNER_REFERENCE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class CornerReference : public Node
{
    public:
        /**
        * member variables according to specification
        */
        int     mId;

    public:
        /**
        * constructor
        */
        explicit CornerReference();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit CornerReference( CornerReference *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~CornerReference();
        
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

#endif /* _OPENDRIVE_CORNER_REFERENCE_HH */
