/* ===================================================
 *  file:       OdrReference.hh
 * ---------------------------------------------------
 *  purpose:	Reference information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_REFERENCE_HH
#define _OPENDRIVE_REFERENCE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"
#include "OdrSignal.hh"
#include "OdrObject.hh"

namespace OpenDrive
{

class Reference : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string mElementType;
        std::string mElementId;
        std::string mType;
        
        Object* mRefObject;
        Signal* mRefSignal;

    public:
        /**
        * constructor
        */
        explicit Reference();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Reference( Reference *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Reference();
        
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

#endif /* _OPENDRIVE_REFERENCE_HH */
