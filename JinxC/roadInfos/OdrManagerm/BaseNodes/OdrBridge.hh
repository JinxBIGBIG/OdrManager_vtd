/* ===================================================
 *  file:       OdrBridge.hh
 * ---------------------------------------------------
 *  purpose:	Bridge information
 * ---------------------------------------------------
 *  first edit:	29.10.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_BRIDGE_HH
#define _OPENDRIVE_BRIDGE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Bridge : public Node 
{
    public:
        /**
        * member variables according to specification
        */
        double        mS;
        double        mLength;
        std::string   mName;
        unsigned int  mId;
        std::string   mIdAsString;
        unsigned int  mType;

    public:
        /**
        * constructor
        */
        explicit Bridge();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Bridge( Bridge *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Bridge();
        
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

#endif /* _OPENDRIVE_BRIDGE_HH */
