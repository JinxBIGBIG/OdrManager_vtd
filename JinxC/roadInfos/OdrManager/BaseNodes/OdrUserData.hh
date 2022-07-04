/* ===================================================
 *  file:       OdrUserData.hh
 * ---------------------------------------------------
 *  purpose:	ancillary user data
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_USER_DATA_HH
#define _OPENDRIVE_USER_DATA_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class UserData : public Node
{
    public:
        /**
        * member variables according to specification
        */
        void* mData;
        
        /**
        * size of user data
        */
        unsigned long mSize;
        
        /**
        * the code of the data
        */
        std::string mCode;
        
        /**
        * the actual data
        */
        std::string mValue;

    public:
        /**
        * constructor
        */
        explicit UserData();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit UserData( UserData *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~UserData();
        
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
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_USER_DATA_HH */
