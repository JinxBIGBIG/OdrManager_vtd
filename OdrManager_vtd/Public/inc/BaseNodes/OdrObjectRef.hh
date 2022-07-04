/* ===================================================
 *  file:       OdrObjectRef.hh
 * ---------------------------------------------------
 *  purpose:	ObjectRef information
 * ---------------------------------------------------
 *  first edit:	02.02.2017 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_OBJECT_REF_HH
#define _OPENDRIVE_OBJECT_REF_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declarations
class Object;

class ObjectRef : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double        mS;
        double        mT;
        unsigned int  mId;
        std::string   mIdAsString;
        unsigned char mDir;
        double        mZ;
        double        mLength;
        int           mMinLane;
        int           mMaxLane;

        /**
        * additional variables for easier navigation
        */
        Object *mObject;

    public:
        /**
        * constructor
        */
        explicit ObjectRef();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit ObjectRef( ObjectRef *rhs );

        /**
        * create a reference from a given object
        *  @param  obj source node which is to be copied
        */
        explicit ObjectRef( Object *obj );

        /** 
        * Destroy the class. 
        */
        virtual ~ObjectRef();
        
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
        * get the s co-ordinate of the node
        * @return s value
        */
        virtual const double & getS() const;

        /**
        * pre-calculate data during the preparation stage
        */
        virtual void calcPrepareData();

};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_OBJECT_REF_HH */
