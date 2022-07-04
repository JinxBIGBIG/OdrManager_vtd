/* ===================================================
 *  file:       OdrSignal.hh
 * ---------------------------------------------------
 *  purpose:	Signal information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_SIGNAL_HH
#define _OPENDRIVE_SIGNAL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declaration
class Controller;
class ControlEntry;
class Object;

class Signal : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double        mS;
        double        mT;
        unsigned int  mId;
        std::string   mIdAsString;
        std::string   mName;
        unsigned char mDynamic;
        unsigned char mDir;
        double        mZ;
        /* unsigned char mCountry; */ /* deprecated in version 1.5.0 */
        int           mType;
        std::string   mTypeAsString;
        int           mSubType;
        std::string   mSubTypeAsString;
        double        mValue;
        unsigned int  mState;   // state of the signal, if it's dynamic
        double        mReadability;
        double        mOcclusion;
        int           mMinLane;
        int           mMaxLane;
        
        // new in ODR 1.4
        unsigned int  mUnit;
        double        mHeight;
        double        mWidth;
        std::string   mText;
        double        mOffsetHdg;
        double        mPitch;
        double        mRoll;
        
        std::string   mCountry;         /** @version 1.5.0 */
        std::string   mCountryRevision; /** @version 1.5.0 */
        
        /**
        * additional variables for internal use
        */
        double          mInX;          // inertial x position
        double          mInY;          // inertial y position
        double          mInZ;          // inertial z position
        double          mInH;          // inertial heading
        Controller*     mController;   // pointer to signal controller
        ControlEntry*   mControlEntry; // pointer to signal control entry
        
        Object*         mLinkedObject; // ticket #8668
        
   private:
       std::string mLinkedObjectId;

    public:
        /**
        * constructor
        */
        explicit Signal();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Signal( Signal *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Signal();
        
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
        * get the s co-ordinate of the node
        * @return s value
        */
        virtual const double & getS() const;
         
        /**
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();
        
        /**
        * get a copy of the node in base class type
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * set the controller handling this signal
        * @param ctrl   pointer to the signal's controller
        */
        void setController( Node *ctrl );
        
        /**
        * set the entry of the controller handling this signal
        * @param ctrl   pointer to the controller's entry
        */
        void setControlEntry( Node *ctrl );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_SIGNAL_HH */
