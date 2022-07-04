/* ===================================================
 *  file:       OdrHeader.hh
 * ---------------------------------------------------
 *  purpose:	header information
 * ---------------------------------------------------
 *  first edit:	01.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_HEADER_HH
#define _OPENDRIVE_HEADER_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Header : public Node
{
    public:
        /**
        * member variables according to specification
        */
        unsigned short mRevMajor;
        unsigned short mRevMinor;
        std::string    mDbName;
        double         mVersion;
        std::string    mTimeDate;
        double         mNorth;
        double         mSouth;
        double         mEast;
        double         mWest;
        std::string    mVendor;
        unsigned int   mMaxRoad;
        unsigned int   mMaxJunc;
        unsigned int   mMaxPrg;
        double         mOrigLat;
        double         mOrigLong;
        double         mOrigHdg;
        double         mOrigHeight;

    public:
        /**
        * constructor
        */
        explicit Header();

        /** 
        * Destroy the class. 
        */
        virtual ~Header();
        
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
        
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_HEADER_HH */
