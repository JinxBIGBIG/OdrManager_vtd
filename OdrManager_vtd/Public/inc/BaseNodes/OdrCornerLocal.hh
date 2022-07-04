/* ===================================================
 *  file:       OdrCornerLocal.hh
 * ---------------------------------------------------
 *  purpose:	CornerLocal information
 * ---------------------------------------------------
 *  first edit:	08.08.2014 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CORNER_LOCAL_HH
#define _OPENDRIVE_CORNER_LOCAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class CornerLocal : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double  mU;
        double  mV;
        double  mZ;
        double  mHeight;
        int     mId;

    public:
        /**
        * constructor
        */
        explicit CornerLocal();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit CornerLocal( CornerLocal *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~CornerLocal();
        
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

#endif /* _OPENDRIVE_CORNER_LOCAL_HH */
