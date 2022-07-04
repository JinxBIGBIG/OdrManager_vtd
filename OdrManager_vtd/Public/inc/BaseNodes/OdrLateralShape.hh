/* ===================================================
 *  file:       OdrLateralShape.hh
 * ---------------------------------------------------
 *  purpose:	LateralShape information
 * ---------------------------------------------------
 *  first edit:	03.03.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_LATERAL_SHAPE_HH
#define _OPENDRIVE_LATERAL_SHAPE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
class LateralShape : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mS;
        double mT;
        double mA;
        double mB;
        double mC;
        double mD;
        
        /**
        * additional variables for higher performance
        */
        double mSEnd;
        double mTEnd;

    public:
        /**
        * constructor
        */
        explicit LateralShape();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit LateralShape( LateralShape *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~LateralShape();
        
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
        * get the LateralShape at a given s position
        * @param s  track co-ordinate
        * @param t  lateral track co-ordinate
        * @return elevation in [m]
        */
        double st2z( const double & s, const double & t );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_LATERAL_SHAPE_HH */
