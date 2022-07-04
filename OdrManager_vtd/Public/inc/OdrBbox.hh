/* ===================================================
 *  file:       OdrBbox.hh
 * ---------------------------------------------------
 *  purpose:	bounding box methods
 * ---------------------------------------------------
 *  first edit:	09.04.2008 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_BBOX_HH
#define _OPENDRIVE_BBOX_HH

// ====== INCLUSIONS ======
#include <iostream>

namespace OpenDrive
{

class Bbox
{
    private:
 
        /**
        * bounding box parameters
        */
        double mXMin;
        double mYMin;
        double mXMax;
        double mYMax;
        
        /**
        * is the box initialized?
        */
        bool mInitialized;
        
    public:
        /**
        * primitive constructor
        */
        explicit Bbox();

        /**
        * copy constructor
        *   @param  rhs      object from which to copy
        */
        explicit Bbox( Bbox *rhs  );

        /**
        * destructor
        */
        virtual ~Bbox();
        
        /**
        * set the extents of the bounding box
        * @param xMin   minimum x value
        * @param yMin   minimum y value
        * @param xMax   maximum x value
        * @param yMax   maximum y value
        */
        void set( const double & xMin, const double & yMin, const double & xMax, const double & yMax );

        /**
        * add another bounding box and get resulting box
        *   @param  box object from which to get additional extents
        */
        void add( Bbox *box );
        
        /**
        * add the extents of another bounding box
        * @param xMin   minimum x value
        * @param yMin   minimum y value
        * @param xMax   maximum x value
        * @param yMax   maximum y value
        */
        void add( const double & xMin, const double & yMin, const double & xMax, const double & yMax );
        
        /**
        * get the extents of the bounding box
        * @param xMin   minimum x value
        * @param yMin   minimum y value
        * @param xMax   maximum x value
        * @param yMax   maximum y value
        */
        void get( double & xMin, double & yMin, double & xMax, double & yMax );

        /**
        * check whether a point is contained in the bounding box
        * @param x   queried x position
        * @param y  queried y position
        * @return true if piont is in box
        */
        bool containsXY( const double & x, const double & y );
        
        /**
        * get the center and the circumscribing radius of the bounding box
        * @param x  center x position
        * @param y  center y position
        * @param radius radius of the circle circumscribing the entire box in the x/y plane
        */
        void getCtrAndRadius( double & x, double & y, double & radius );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_BBOX_HH */
