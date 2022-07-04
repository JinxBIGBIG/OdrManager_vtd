/* ===================================================
 *  file:       OdrObjectOutline.hh
 * ---------------------------------------------------
 *  purpose:	ObjectOutline information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_OBJECT_OUTLINE_HH
#define _OPENDRIVE_OBJECT_OUTLINE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class ObjectOutline : public Node
{
    public:
        /**
        * member variables according to specification
        */
        int         mId;
        int         mFillType;
        bool        mOuter;
        bool        mClosed;
        std::string mLaneType;
 
    public:
        /**
        * constructor
        */
        explicit ObjectOutline();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit ObjectOutline( ObjectOutline *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~ObjectOutline();
        
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

#endif /* _OPENDRIVE_OBJECT_OUTLINE_HH */
