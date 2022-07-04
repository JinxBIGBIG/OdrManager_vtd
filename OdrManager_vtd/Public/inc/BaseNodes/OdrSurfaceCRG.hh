/* ===================================================
 *  file:       OdrSurfaceCRG.hh
 * ---------------------------------------------------
 *  purpose:	Surface information
 * ---------------------------------------------------
 *  first edit:	23.01.2012 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_SURFACE_CRG_HH
#define _OPENDRIVE_SURFACE_CRG_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declaration

class SurfaceCRG : public Node
{
    public:
        static const unsigned int sModeGenuine;
        static const unsigned int sModeAttached;
        static const unsigned int sModeAttached0;
        static const unsigned int sModeGlobal;
        
        static const unsigned int sPurposeElevation;
        static const unsigned int sPurposeFriction;
        static const unsigned int sPurposeAny;
        
    public:
        /**
        * member variables according to specification
        */
        std::string    mFileName;
        double         mS;
        double         mSEnd;
        unsigned char  mDir;
        unsigned int   mMode;
        double         mSOffset;
        double         mTOffset;
        double         mZOffset;
        double         mZScale;
        double         mHOffset;
        unsigned short mPurpose;
        
        /**
        * name of parent file, see ticket #5226
        */
        std::string mParentFileName;
        
        /**
        * is this a data set for a junction?
        * @version 1.5.0
        */
        bool mIsInJunction;

    public:
        /**
        * constructor
        */
        explicit SurfaceCRG();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit SurfaceCRG( SurfaceCRG *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~SurfaceCRG();
        
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
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * retrieve z position at given s/t position
        * @param s track co-ordinate
        * @param t lateral distance from element at ds
        * @param z return parameter for z
        * @param p return parameter for pitch
        * @param calcPitch set to true in order to have pitch value calculated
        * @return success/failure
        */
        virtual bool st2zp( const double & s, const double & t, double & z, double & p, bool calcPitch = false );
        
        /**
        * retrieve z position at given convert x/y position
        * @param x x-position
        * @param y y-position
        * @param z return parameter for z
        * @param p return parameter for pitch
        * @param calcPitch set to true in order to have pitch value calculated
        * @return success/failure
        */
        virtual bool xy2zp( const double & x, const double & y, double & z, double & p, bool calcPitch = false );
        
        /**
        * set global search for existing CRG data
        */
        void enableGlobalSearch();
        
        /**
        * invert CRG direction
        */
        void setInverseDirection();
        
    private:
        /**
        * ID of the OpenCRG data set
        */
        int mDataSetId;
        
        /**
        * ID of a contact point for data evaluation
        */
        int mCpId;
        
        /**
        * global search for existing data?
        */
        bool mGlobalSearch;
        
        /**
        * invert evaluation?
        */
        bool mInverseDirection;
        
        /**
        * remember Uu range and v range
        */
        double mUMin;
        double mUMax;
        double mVMin;
        double mVMax;
        
        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_SURFACE_CRG_HH */
