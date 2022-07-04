/* ===================================================
 *  file:       OdrTunnel.hh
 * ---------------------------------------------------
 *  purpose:	Tunnel information
 * ---------------------------------------------------
 *  first edit:	29.10.2006 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_TUNNEL_HH
#define _OPENDRIVE_TUNNEL_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class Tunnel : public Node  // TODO
{
    public:
        /**
        * member variables according to specification
        */
        double        mS;
        double        mLength;
        std::string   mName;
        unsigned int  mId;
        std::string   mIdAsString;
        unsigned int  mType;
        double        mLighting;
        double        mDaylight;

    public:
        /**
        * constructor
        */
        explicit Tunnel();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Tunnel( Tunnel *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Tunnel();
        
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

#endif /* _OPENDRIVE_TUNNEL_HH */
