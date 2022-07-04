/* ===================================================
 *  file:       OdrCrossfall.hh
 * ---------------------------------------------------
 *  purpose:	Crossfall information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_CROSSFALL_HH
#define _OPENDRIVE_CROSSFALL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
class Crossfall : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned int mSide;
        double       mS;
        double       mA;
        double       mB;
        double       mC;
        double       mD;

    public:
        /**
        * constructor
        */
        explicit Crossfall();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Crossfall( Crossfall *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Crossfall();
        
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
        * get the crossfall at a given s position
        * @param s  track co-ordinate
        * @return crossfall in [rad]
        */
        double s2xfall( const double & s );
        
        /**
        * get the first derivative of the corssfall at a given s position
        * @param s  track co-ordinate
        * @return derivative of crossfall in [rad/m]
        */
        double s2xfallDot( const double & s );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_CROSSFALL_HH */
