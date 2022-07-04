/* ===================================================
 *  file:       OdrRepeat.hh
 * ---------------------------------------------------
 *  purpose:	Repeat information
 * ---------------------------------------------------
 *  first edit:	05.01.2015 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_REPEAT_HH
#define _OPENDRIVE_REPEAT_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Repeat : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double mS;
        double mLength;
        double mDistance;
        double mTStart;
        double mTEnd;
        double mZOffsetStart;
        double mZOffsetEnd;
        double mWidthStart;
        double mWidthEnd;
        double mHeightStart;
        double mHeightEnd;
        
        double mLengthStart;  /** @version 1.5.0 */
        double mLengthEnd;    /** @version 1.5.0 */
        double mRadiusStart;  /** @version 1.5.0 */
        double mRadiusEnd;    /** @version 1.5.0 */
        bool   mIsCircular;   /** @version 1.5.0 */

    public:
        /**
        * constructor
        */
        explicit Repeat();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Repeat( Repeat *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Repeat();
        
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
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_REPEAT_HH */
