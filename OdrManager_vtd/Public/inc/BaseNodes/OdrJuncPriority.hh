/* ===================================================
 *  file:       OdrJuncPriority.hh
 * ---------------------------------------------------
 *  purpose:	JuncPriority information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_PRIORITY_HH
#define _OPENDRIVE_JUNC_PRIORITY_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class JuncPriority : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int mPrioId;
        unsigned int mLowPath;

    public:
        /**
        * constructor
        */
        explicit JuncPriority();

        /** 
        * Destroy the class. 
        */
        virtual ~JuncPriority();
        
        /**
        * print the data contents
        */
        virtual void printData() const;
        
        /**
        * read the contents of the node from a given file descriptor
        * @param reader     the reader to be used for reading the data
        * @param noBytes    number of bytes to be read
        * @return failure/success
        */
        bool read( ReaderXML* reader );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_PRIORITY_HH */
