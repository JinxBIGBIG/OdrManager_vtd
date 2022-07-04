/* ===================================================
 *  file:       OdrElevation.hh
 * ---------------------------------------------------
 *  purpose:	Elevation information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ELEVATION_HH
#define _OPENDRIVE_ELEVATION_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Elevation : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mS;
        double mA;
        double mB;
        double mC;
        double mD;

        /**
        * additional variables for higher performance
        */
        double mSEnd;

    public:
        /**
        * constructor
        */
        explicit Elevation();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Elevation( Elevation *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Elevation();
        
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
        * get the elevation at a given s position
        * @param s  track co-ordinate
        * @return elevation in [m]
        */
        double s2z( const double & s );
        
        /**
        * get the pitch angle at a given s position
        * @param s  track co-ordinate
        * @return pitch angle in [rad]
        */
        double s2pitch( const double & s );
        
        /**
        * get the derivative of the pitch angle at a given s position
        * @param s  track co-ordinate
        * @return derivative of pitch angle in [rad/m]
        */
        double s2pitchDot( const double & s );
        
        /**
        * get the 2nd derivative of the elevation at a given s position
        * @param s  track co-ordinate
        * @return 2nd derivative of the elevation
        */
        double s2secondDeriv( const double & s );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * get the curvature of the elevation at a given s position
        * @param s  track co-ordinate
        * @return curvature of the elevation
        */
        double s2curvature( const double & s );
        
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

#endif /* _OPENDRIVE_ELEVATION_HH */
