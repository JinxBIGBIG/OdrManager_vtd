/* ===================================================
 *  file:       OdrObjectMaterial.hh
 * ---------------------------------------------------
 *  purpose:	ObjectMaterial information
 * ---------------------------------------------------
 *  first edit:	03.02.2017 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_OBJECT_MATERIAL_HH
#define _OPENDRIVE_OBJECT_MATERIAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class ObjectMaterial : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mFriction;
        double         mRoughness;
        std::string    mSurface;

    public:
        /**
        * constructor
        */
        explicit ObjectMaterial();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit ObjectMaterial( ObjectMaterial *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~ObjectMaterial();
        
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

#endif /* _OPENDRIVE_OBJECT_MATERIAL_HH */
