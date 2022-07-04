/* ===================================================
 *  file:       OdrJuncController.hh
 * ---------------------------------------------------
 *  purpose:	JuncController information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 *  last mod.:  12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_CONTROLLER_HH
#define _OPENDRIVE_JUNC_CONTROLLER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declaration
class Controller;

class JuncController : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int mId;
        std::string mIdAsString;
        std::string mType;
        
        // advance definitions for Odr 1.3
        std::string  mSequence;
        
        /**
        * additional variables for advanced use
        */
        Controller*  mController;

    public:
        /**
        * constructor
        */
        explicit JuncController();

        /** 
        * Destroy the class. 
        */
        virtual ~JuncController();
        
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
        * set the pointer to the actual controller (which is associated with signals)
        * @param ctrl   pointer to the controller
        */
        void setController( Node *ctrl );
        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_CONTROLLER_HH */
