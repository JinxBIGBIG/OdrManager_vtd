/* ===================================================
 *  file:       OdrGenericNode.hh
 * ---------------------------------------------------
 *  purpose:	child nodes of user data
 * ---------------------------------------------------
 *  first edit:	07.12.2012 by M. Knerr @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_GENERIC_NODE_HH
#define _OPENDRIVE_GENERIC_NODE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"
#include "OdrReaderXML.hh"

namespace OpenDrive
{

class GenericNode : public Node
{
    public:
        ReaderXML::AttribMap    mAttributesMap; //<! stores all attributes as key value pairs

    public:
        /**
        * constructor
        */
        explicit GenericNode( const std::string& rName );

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit GenericNode( GenericNode *rhs );

        /** 
        * Destroy the class.
        */
        virtual ~GenericNode();
        
        /**
        * print the data contents
        */
        virtual void printData() const;
        
        /**
        * read the contents of the node by fetching the complete attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        virtual bool read( ReaderXML* reader );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_GENERIC_NODE_HH */
