/* ===================================================
 *  file:       OdrOffset.hh
 * ---------------------------------------------------
 *  purpose:	Offset information
 * ---------------------------------------------------
 *  first edit:	30.05.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_OFFSET_HH
#define _OPENDRIVE_OFFSET_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Offset : public Node  
{
    public:
        /**
        * member variables according to specification
        */
        double mX;
        double mY;
        double mZ;
        double mH;

    public:
        /**
        * constructor
        */
        explicit Offset();

        /**
        * copy constructor
        * @param  rhs source node which is to be copied
        * @version 1.5.0
        */
        explicit Offset( Offset *rhs );

        /** 
        * Destroy the class. 
        * @version 1.5.0
        */
        virtual ~Offset();
        
        /**
        * print the data contents
        * @version 1.5.0
        */
        virtual void printData() const;
        
       /**
        * read the contents of the node from a prepared attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        * @version 1.5.0
        */
        bool read( ReaderXML* reader );
        
        /**
        * get a copy of the node in base class type
        * @param deep if true, perform a deep copy
        * @version 1.5.0
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_OFFSET_HH */
