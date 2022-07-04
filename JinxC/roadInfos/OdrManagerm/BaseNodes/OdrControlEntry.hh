/* ===================================================
 *  file:       OdrControlEntry.hh
 * ---------------------------------------------------
 *  purpose:	ControlEntry information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CONTROL_ENTRY_HH
#define _OPENDRIVE_CONTROL_ENTRY_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"
#include "OdrSignal.hh"

namespace OpenDrive
{

class ControlEntry : public OpenDrive::Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int mSigId;
        std::string  mSigIdAsString;
        unsigned int mType;
        
        /**
        * additional variables for advanced use
        */
        Signal*  mSignal;

    public:
        /**
        * constructor
        */
        explicit ControlEntry();

        /** 
        * Destroy the class. 
        */
        virtual ~ControlEntry();
        
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
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_CONTROL_ENTRY_HH */
