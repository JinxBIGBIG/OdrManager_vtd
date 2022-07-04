/* ===================================================
 *  file:       OdrGeoHeader.hh
 * ---------------------------------------------------
 *  purpose:	GeoHeader information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_GEO_HEADER_HH
#define _OPENDRIVE_GEO_HEADER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

// forward declarations
class Bbox;

class GeoHeader : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mS;
        double mX;
        double mY;
        double mH;
        double mLength;
        
        /**
        * additional variables for internal use
        */
        double mSEnd;
        double mXEnd;
        double mYEnd;
        double mHEnd;
        double mXCtr;
        double mYCtr;
        
        /**
        * check geometry upon initialization?
        */
        bool mCheckGeometry;
        
        /**
        * curvature at beginning and end of node
        */
        double mCurv;
        double mCurvEnd;
        bool   mUseCurvApprox;
        
        /**
        * enable / disable curvature approximation
        */
        bool mEnableCurvApprox;

    private:
        /**
        * bounding box of road
        */
        Bbox *mBbox;

    public:
        /**
        * constructor
        */
        explicit GeoHeader();

        /** 
        * Destroy the class. 
        */
        virtual ~GeoHeader();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit GeoHeader( GeoHeader *rhs );
        
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
        * set the x and y end value for the geometry element
        * @param xEnd   x-position at end of element
        * @param yEnd   y-position at end of element
        * @param hEnd   heading at end of element
        */
        void setPosEndValue( const double & xEnd, const double & yEnd, const double & hEnd );
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * calculate the bounding box
        * @param width  road width to be taken into account
        */
        void calcBoundingBox( const double & width );

        /**
        * get the bounding box
        */
        Bbox* getBoundingBox();

        /**
        * check the bounding box of the element contains a given point
        * @param x  x-position
        * @param y  y-position
        * @return   true if position is in bounding box
        */
        bool inBoundingBox( const double & x, const double & y );
        
        /**
        * get the maximum absolute curvature in a given range of the reference line
        * @param sMin  minimum s position
        * @param sMax  maximum s position
        * @return maximum curvature (max in terms of abs value) 
        */
        virtual double getMaxCurvatureInRange( const double & sMin, const double & sMax );
        
        /**
        * calculate the respective data after the preparation stage 
        */
        virtual void calcPostPrepareData();
        
        /**
        * apply inertial transformation operation on a single node
        * @param dx inertial x transformation (shift) [m]
        * @param dy inertial y transformation (shift) [m]
        * @param dz inertial z transformation (shift) [m]
        * @param dh inertial heading transformation (rotation) [rad]
        * @version 1.5.0
        */
        virtual void applyTransformation( const double & dx, const double & dy, const double &dz, const double & dh );
        
    private:
        /**
        * prepare the node with initial calculations
        */
        void init();
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_GEO_HEADER_HH */
