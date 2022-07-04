/* ===================================================
 *  file:       OdrRawDataDesc.hh
 * ---------------------------------------------------
 *  purpose:	ancillary raw data description
 * ---------------------------------------------------
 *  first edit:	03.06.2019 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_RAW_DATA_DESC_HH
#define _OPENDRIVE_RAW_DATA_DESC_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{

class RawDataDesc : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string mDate;
        std::string mSource;
        std::string mSourceComment;
        std::string mPostProcessing;
        std::string mPostProcessingComment;

    public:
        /**
        * constructor
        */
        explicit RawDataDesc();

        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit RawDataDesc( RawDataDesc *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~RawDataDesc();
        
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

#endif /* _OPENDRIVE_RAW_DATA_DESC_HH */
