/* ===================================================
 *  file:       OdrRoadType.hh
 * ---------------------------------------------------
 *  purpose:	RoadType information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_TYPE_HH
#define _OPENDRIVE_ROAD_TYPE_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadType : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double       mStartPos;
        unsigned int mType;
        std::string  mCountry;  /** @version 1.5.0 */
        
        /**
        * additional variables for faster computation
        */
        double mEndPos;

    public:
        /**
        * constructor
        */
        explicit RoadType();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadType( RoadType *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadType();
        
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

#endif /* _OPENDRIVE_ROAD_TYPE_HH */
