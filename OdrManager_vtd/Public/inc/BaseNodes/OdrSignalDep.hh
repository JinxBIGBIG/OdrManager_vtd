/* ===================================================
 *  file:       OdrSignalDep.hh
 * ---------------------------------------------------
 *  purpose:	SignalDep information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_SIGNAL_DEP_HH
#define _OPENDRIVE_SIGNAL_DEP_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class SignalDep : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int mId;
        std::string  mIdAsString;
        unsigned int mType;

    public:
        /**
        * constructor
        */
        explicit SignalDep();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit SignalDep( SignalDep *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~SignalDep();
        
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

#endif /* _OPENDRIVE_SIGNAL_DEP_HH */
