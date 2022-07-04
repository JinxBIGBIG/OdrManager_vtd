/* ===================================================
 *  file:       OdrLaneMaterial.hh
 * ---------------------------------------------------
 *  purpose:	LaneMaterial information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LANE_MATERIAL_HH
#define _OPENDRIVE_LANE_MATERIAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class LaneMaterial : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double         mOffset;
        unsigned int   mCode;
        double         mFriction;
        double         mRoughness;
        std::string    mSurface;

    public:
        /**
        * constructor
        */
        explicit LaneMaterial();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LaneMaterial( LaneMaterial *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LaneMaterial();
        
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

#endif /* _OPENDRIVE_LANE_MATERIAL_HH */
