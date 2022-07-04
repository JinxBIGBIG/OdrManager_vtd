/* ===================================================
 *  file:       OdrJuncHeader.hh
 * ---------------------------------------------------
 *  purpose:	road header information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_HEADER_HH
#define _OPENDRIVE_JUNC_HEADER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
class JuncLink;
class JuncController;
class Bbox;
class SurfaceCRG;

class JuncHeader : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mName;
        unsigned int mId;
        std::string  mIdAsString;
        bool         mIsVirtual;

    public:
        /**
        * constructor
        */
        explicit JuncHeader();
        
        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit JuncHeader( JuncHeader *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~JuncHeader();
        
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
        * get the pointer to the first link in the junction
        * @return   pointer to first link
        */
        JuncLink* getFirstLink();
        
        /**
        * get the pointer to the first controller in the junction
        * @return   pointer to first controller
        */
        JuncController* getFirstController();

        /**
        * calculate the respective data after the preparation stage 
        */
        virtual void calcPostPrepareData();
        
        /**
        * get the bounding box of all paths in the junction
        */
        Bbox* getBoundingBox();
        
        /**
        * get the center and the circumscribing radius of the bounding box
        * @param x  center x position
        * @param y  center y position
        * @param radius radius of the circle circumscribing the entire box in the x/y plane
        */
        void getCtrAndRadius( double & x, double & y, double & radius );

        /**
        * check whether the junction has an associated SurfaceCRG node
        * @param purpose    the purpose (mask) for which the surface data may be used
        * @return surfaceCRG node matching the junction
        */
        SurfaceCRG* getSurface( const unsigned short & purpose );

    private:
        /**
        * calculate the bounding box
        */
        void calcBoundingBox();

    private:
        /**
        * bounding box of all paths in the junction
        */
        Bbox *mBbox;
        
        /** 
        * additional surface data available?
        */
        bool mHasSurfaceData;
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_HEADER_HH */
