/* ===================================================
 *  file:       OdrController.hh
 * ---------------------------------------------------
 *  purpose:	Controller information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CONTROLLER_HH
#define _OPENDRIVE_CONTROLLER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
//forward declarations
class ControlEntry;

class Controller : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mName;
        unsigned int mId;
        std::string  mIdAsString;

    public:
        /**
        * constructor
        */
        explicit Controller();

        /** 
        * Destroy the class. 
        */
        virtual ~Controller();
        
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
        * get the pointer to the first control entry of this controller
        * @return pointer to first control entry
        */
        ControlEntry* getFirstEntry();
        
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_CONTROLLER_HH */
