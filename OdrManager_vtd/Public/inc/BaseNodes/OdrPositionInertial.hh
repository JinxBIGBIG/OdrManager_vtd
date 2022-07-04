/* ===================================================
 *  file:       OdrPositionInertial.hh
 * ---------------------------------------------------
 *  purpose:	PositionInertial information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_POSITION_INERTIAL_HH
#define _OPENDRIVE_POSITION_INERTIAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class PositionInertial : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mX;
        double mY;
        double mZ;
        double mHdg;
        double mPitch;
        double mRoll;       

    public:
        /**
        * constructor
        */
        explicit PositionInertial();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit PositionInertial( PositionInertial *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~PositionInertial();
        
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

#endif /* _OPENDRIVE_POSITION_INERTIAL_HH */
