/* ===================================================
 *  file:       OdrGeoNode.hh
 * ---------------------------------------------------
 *  purpose:	Parent node for all geometry nodes
 *              contains evaluation methods
 * ---------------------------------------------------
 *  first edit:	03.12.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_GEO_NODE_HH
#define _OPENDRIVE_GEO_NODE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

// ====== DEFINITIONS ======

namespace OpenDrive
{

// forward declarations
class GeoHeader;
    
class GeoNode : public Node
{
    public:
        static const unsigned int sMaxSearchLevel = 2;
        
    public:
        /**
        * constructor
        *   @param  name    name under which data is managed
        */
        explicit GeoNode( const std::string & name );

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit GeoNode( GeoNode *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~GeoNode();
        
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
        * read the contents of the node from a given XML data structure
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        virtual bool readIntern( ReaderXML* reader ) = 0;
        
        /**
        * convert s/t position into x/y position
        * @param s track co-ordinate
        * @param t lateral distance from element at ds
        * @param x return parameter for x
        * @param y return parameter for y
        * @param h return parameter for heading
        * @return success/failure
        */
        virtual bool st2xyh( const double & s, const double & t, double & x, double & y, double & h );
        
        /**
        * convert s position into heading
        * @param s track co-ordinate
        * @param h return parameter for heading
        * @return success/failure
        */
        virtual bool s2h( const double & s, double & h );
        
        /**
        * convert ds/t position into dx/dy position
        * @param ds delta s from begin of element
        * @param dx return parameter for delta x
        * @param dy return parameter for delta y
        * @param dh return parameter for delta heading
        * @return success/failure
        */
        virtual bool ds2dxdydh( const double & ds, double & dx, double & dy, double & dh );
        
        /**
        * convert ds position into dh position
        * @param ds delta s from begin of element
        * @param dh return parameter for delta heading
        * @return success/failure
        */
        virtual bool ds2dh( const double & ds, double & dh );
        
        /**
        * get the curvature at a given s position
        * @param s track co-ordinate
        * @return curvature
        */
        virtual double s2curvature( const double & s );
        
        /**
        * check whether the element contains a given inertial position
        * @param x  x-position
        * @param y  y-position
        * @return   true if position is contained in given element
        */
        virtual bool containsPos( const double & x, const double & y );

        /**
        * convert x/y position into s/t position
        * @param x return parameter for x
        * @param y return parameter for y
        * @param s track co-ordinate
        * @param t lateral distance from element at ds
        * @return success/failure
        */
        virtual bool xy2st( const double & x, const double & y, double & s, double & t );
        
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine of the base class
        */
        virtual void calcPrepareData();
        
        /**
        * calculate the bounding box
        * @param maxT   maximum t value to be taken into account
        * @param xMin   minimum x value
        * @param yMin   minimum y value
        * @param xMax   maximum x value
        * @param yMax   maximum y value
        */
        virtual void calcBoundingBox( const double & maxT, double & xMin, double & yMin, double & xMax, double & yMax );
        
        /**
        * force the setting of the header node
        * @param hdr header node which is to be used
        */
        virtual void setHeader( GeoHeader* hdr );
        
        /**
        * get the maximum absolute curvature in a given range of the reference line
        * @param dsMin  minimum local s position
        * @param dsMax  maximum local s position
        * @return maximum curvature (max in terms of abs value) 
        */
        virtual double getMaxCurvatureInRange( const double & dsMin, const double & dsMax );
        
    protected:
        /**
        * keep a type-casted reference to the parent node
        */
        GeoHeader *mHdr;
        
        double mStartCval;
        double mStartSval;
        double mDs;
        
    protected:
        /**
        * trim a given ds position to the valid ds positions within
        * the element, cutting off overruns at lower and upper limit\
        * @param ds the original ds value
        * @return the trimmed ds value
        */
        const double & ds2validDs( const double & ds );

        /**
        * check whether a sub-section of a geometry element contains a given point
        * @param x x-position
        * @param y y-position
        * @param xyhsLeft    x/y/heading/s left side
        * @param xyhsRight   x/y/heading/s right side
        * @param level  iteration level
        * @return   true if position is contained in given element section
        */
        bool pointIsInSection( const double & x, const double & y, 
                               const double & xLeft, const double & yLeft, 
                               const double & hLeft, const double & sLeft,
                               const double & xRight, const double & yRight, 
                               const double & hRight, const double & sRight,
                               unsigned int level );
                               
        /**
        * check whether a point is between two given point/dir pairs
        * @param x x-position
        * @param y y-position
        * @param xyhLeft    x/y/heading left side
        * @param xyhRight   x/y/heading right side
        * @return   true if position is contained in given element section
        */
        bool pointIsBetween( const double & x,      const double & y, 
                             const double & xLeft,  const double & yLeft,  const double & hLeft,
                             const double & xRight, const double & yRight, const double & hRight );
        
    protected:
        /**
        * algorithm to determine a foot-point on the element for a
        * given point in the x/y plane
        * @param x  x-position under investigation
        * @param y  y-position under investigation
        * @param s  returned s-value of foot point
        * @param xs returned x-value of foot point
        * @param ys returned y-value of foot point
        * @param phi returned heading of foot point
        * @param xyhsLeft    x/y/heading/s left side
        * @param xyhsRight   x/y/heading/s right side
        * @return success/failure
        */
        virtual bool secantAlgorithm( const double & x, const double & y,  
                                      double & s,  double & xs, 
                                      double & ys, double & phi,
                                      const double & xLeft, const double & yLeft, 
                                      const double & hLeft, const double & sLeft,
                                      const double & xRight, const double & yRight, 
                                      const double & hRight, const double & sRight );
    
        /**
        * calculate the side of a point with respect to the geometric element 
        * @param xp foot-point x
        * @param yp foot-point y
        * @param phi direction at foot-point
        * @param x x-position under investigation
        * @param y y-position under investigation 
        */
        double getSideOfPoint( const double & xp, const double & yp, 
                               const double & phi, const double & x, const double & y );

    private:
        double mDs0;        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_GEO_ARC_HH */
