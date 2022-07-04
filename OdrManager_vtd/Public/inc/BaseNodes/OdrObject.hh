/* ===================================================
 *  file:       OdrObject.hh
 * ---------------------------------------------------
 *  purpose:	Object information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_OBJECT_HH
#define _OPENDRIVE_OBJECT_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declaration
class Repeat;

class Object : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string   mType;
        std::string   mName;
        unsigned int  mId;
        std::string   mIdAsString;
        double        mS;
        double        mT;
        double        mZ;
        double        mLength;
        unsigned char mDir;
        double        mSizeX;
        double        mSizeY;
        double        mRadius;
        double        mSizeZ;
        double        mHdg;
        double        mPitch;
        double        mRoll;
        double        mValue;
        bool          mHasOutline;
        double        mInX;
        double        mInY;
        double        mInZ;
        double        mInH;
        bool          mHasRepeat;
        std::string   mSubType;
        bool          mIsDynamic;
        bool          mIsCircular;
        

    public:
        /**
        * constructor
        */
        explicit Object();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Object( Object *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Object();
        
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
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();
        
        /**
        * get the pointer to the first repeat record (as child)
        * @return pointer to the first repeat record
        */
        Repeat* getFirstRepeat();     
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_OBJECT_HH */
