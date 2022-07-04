/* ===================================================
 *  file:       OdrSignalRef.hh
 * ---------------------------------------------------
 *  purpose:	SignalRef information
 * ---------------------------------------------------
 *  first edit:	29.10.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_SIGNAL_REF_HH
#define _OPENDRIVE_SIGNAL_REF_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declarations
class Signal;

class SignalRef : public Node
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
        
        int           mMinLane;
        int           mMaxLane;

        /**
        * additional variables for easier navigation
        */
        Signal *mSignal;

    public:
        /**
        * constructor
        */
        explicit SignalRef();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit SignalRef( SignalRef *rhs );

        /**
        * create a reference from a given signal
        *  @param  rhs source node which is to be copied
        */
        explicit SignalRef( Signal *sig );

        /** 
        * Destroy the class. 
        */
        virtual ~SignalRef();
        
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

#endif /* _OPENDRIVE_SIGNAL_DEP_HH */
