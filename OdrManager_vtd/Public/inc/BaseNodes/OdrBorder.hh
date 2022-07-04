/* ===================================================
 *  file:       OdrBorder.hh
 * ---------------------------------------------------
 *  purpose:	Border information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_BORDER_HH
#define _OPENDRIVE_BORDER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Border : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double      mWidth;
        std::string mType;
        int         mOutlineId;
        bool        mUseCompleteOutline;

    public:
        /**
        * constructor
        */
        explicit Border();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Border( Border *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Border();
        
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

#endif /* _OPENDRIVE_CORNER_BORDER_HH */
