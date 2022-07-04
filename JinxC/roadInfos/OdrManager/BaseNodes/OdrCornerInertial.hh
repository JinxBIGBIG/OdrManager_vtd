/* ===================================================
 *  file:       OdrCornerInertial.hh
 * ---------------------------------------------------
 *  purpose:	CornerInertial information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CORNER_INERTIAL_HH
#define _OPENDRIVE_CORNER_INERTIAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class CornerInertial : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double  mX;
        double  mY;
        double  mZ;
        double  mHeight;
        int     mId;

    public:
        /**
        * constructor
        */
        explicit CornerInertial();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit CornerInertial( CornerInertial *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~CornerInertial();
        
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
        
        /**
        * apply inertial transformation operation on a single node
        * @param dx inertial x transformation (shift) [m]
        * @param dy inertial y transformation (shift) [m]
        * @param dz inertial z transformation (shift) [m]
        * @param dh inertial heading transformation (rotation) [rad]
        * @version 1.5.0
        */
        virtual void applyTransformation( const double & dx, const double & dy, const double &dz, const double & dh );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_CORNER_INERTIAL_HH */
