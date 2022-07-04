/* ===================================================
 *  file:       OdrRoadLink.hh
 * ---------------------------------------------------
 *  purpose:	road link information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_ROAD_LINK_HH
#define _OPENDRIVE_ROAD_LINK_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RoadLink : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned short mType;
        unsigned short mElemType;
        unsigned int   mElemId;
        std::string    mElemIdAsString;
        unsigned char  mElemDir;
        unsigned short mSide;   // for neighbor only!
        double         mElemS;  /** @version 1.5.0 */
        
        // additional information
        bool           mVirtualLink;

    public:
        /**
        * constructor
        */
        explicit RoadLink();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RoadLink( RoadLink *rhs );

        /**
        * constructor
        * @param type define the type of the link
        */
        explicit RoadLink( unsigned short type );

        /** 
        * Destroy the class. 
        */
        virtual ~RoadLink();
        
        /**
        * print the data contents
        */
        virtual void printData() const;

        unsigned int getElemId() const;
        unsigned short getElemType() const;
        std::string getElemIdStr() const; 
        
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

#endif /* _OPENDRIVE_ROAD_LINK_HH */
