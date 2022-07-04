/* ===================================================
 *  file:       OdrJuncRef.hh
 * ---------------------------------------------------
 *  purpose:	JuncRef information
 * ---------------------------------------------------
 *  first edit:	03.03.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNCTION_REFERENCE_HH
#define _OPENDRIVE_JUNCTION_REFERENCE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declarations
class JuncHeader;

class JuncRef : public Node
{
    public:
        /**
        * member variables according to specification
        */
        JuncHeader *mJuncHeader;
        unsigned int mId;
        std::string  mIdAsString;
        
    public:
        /**
        * constructor
        */
        explicit JuncRef();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit JuncRef( JuncRef *rhs );

        /**
        * create a reference from a given junction
        *  @param  junc source node which is to be referenced
        */
        explicit JuncRef( JuncHeader *junc );

        /** 
        * Destroy the class. 
        */
        virtual ~JuncRef();
        
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
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNCTION_REFERENCE_HH */
