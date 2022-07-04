/* ===================================================
 *  file:       OdrMarkings.hh
 * ---------------------------------------------------
 *  purpose:	Marking Group information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_MARKINGS_HH
#define _OPENDRIVE_MARKINGS_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Markings : public Node
{
    public:
        /**
        * constructor
        */
        explicit Markings();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Markings( Markings *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Markings();
        
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

#endif /* _OPENDRIVE_MARKINGS_HH */
