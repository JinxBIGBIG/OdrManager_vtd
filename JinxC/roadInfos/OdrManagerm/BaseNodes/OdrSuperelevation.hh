/* ===================================================
 *  file:       OdrSuperelevation.hh
 * ---------------------------------------------------
 *  purpose:	Superelevation information
 * ---------------------------------------------------
 *  first edit:	29.10.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_SUPERELEVATION_HH
#define _OPENDRIVE_SUPERELEVATION_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
class Superelevation : public Node
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
        explicit Superelevation();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Superelevation( Superelevation *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Superelevation();
        
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
        * get the superelevation at a given s position
        * @param s  track co-ordinate
        * @return Superelevation in [rad]
        */
        double s2xfall( const double & s );
        
        /**
        * get the first derivative of the superelevation at a given s position
        * @param s  track co-ordinate
        * @return derivative of Superelevation in [rad/m]
        */
        double s2xfallDot( const double & s );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_SUPERELEVATION_HH */
