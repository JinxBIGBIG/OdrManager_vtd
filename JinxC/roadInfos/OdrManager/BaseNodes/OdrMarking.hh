/* ===================================================
 *  file:       OdrMarking.hh
 * ---------------------------------------------------
 *  purpose:	Marking information
 * ---------------------------------------------------
 *  first edit:	02.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_MARKINGS_MARKING_HH
#define _OPENDRIVE_MARKINGS_MARKING_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Marking : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mSide;
        std::string  mWeight;
        double       mWidth;
        unsigned int mColor;
        double       mOffsetZ;
        double       mSpaceLength;
        double       mLineLength;
        double       mStartOffset;
        double       mStopOffset;

    public:
        /**
        * constructor
        */
        explicit Marking();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Marking( Marking *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Marking();
        
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
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_MARKINGS_MARKING_HH */
