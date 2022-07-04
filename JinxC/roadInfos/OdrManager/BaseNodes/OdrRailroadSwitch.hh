/* ===================================================
 *  file:       OdrRailroadSwitch.hh
 * ---------------------------------------------------
 *  purpose:	railroad switch information
 * ---------------------------------------------------
 *  first edit:	14.03.2012 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_RAILROAD_SWITCH_HH
#define _OPENDRIVE_RAILROAD_SWITCH_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
// forward declarations
class GenericNode;
    
class RailroadSwitch : public Node
{
    public:
        /**
        * member variables according to specification
        */
        double        mS;
        unsigned int  mId;
        std::string   mIdAsString;
        std::string   mName;
        unsigned int  mSideTrackId;
        std::string   mSideTrackIdAsString;
        double        mSideTrackS;
        char          mSideTrackDir;
        unsigned int  mState;   
        char          mDir;
        bool          mIsDynamic;

    public:
        /**
        * constructor
        */
        explicit RailroadSwitch();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RailroadSwitch( RailroadSwitch *rhs );

        /**
        * copy constructor
        *  @param  src generic node from which to instantiate the data
        */
        explicit RailroadSwitch( GenericNode *src );

        /** 
        * Destroy the class. 
        */
        virtual ~RailroadSwitch();
        
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
        * set reader for next read sequence (main track information)
        */
        void readMainTrack();
        
        /**
        * set reader for next read sequence (side track information)
        */
        void readSideTrack();
        
    private:
        /**
        * reader modes
        */
        bool mReadMainTrack;
        bool mReadSideTrack;
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_RAILROAD_SWITCH_HH */
