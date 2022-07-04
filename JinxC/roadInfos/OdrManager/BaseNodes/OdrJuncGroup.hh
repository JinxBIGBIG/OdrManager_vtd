/* ===================================================
 *  file:       OdrJuncGroup.hh
 * ---------------------------------------------------
 *  purpose:	road header information
 * ---------------------------------------------------
 *  first edit:	12.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_JUNC_GROUP_HH
#define _OPENDRIVE_JUNC_GROUP_HH

// ====== INCLUSIONS ======
#include "OdrNode.hh"

namespace OpenDrive
{
    
class Bbox;
class JuncRef;

class JuncGroup : public Node
{
    public:
        /**
        * member variables according to specification
        */
        std::string  mName;
        unsigned int mId;
        std::string  mIdAsString;
        std::string  mType;

    public:
        /**
        * constructor
        */
        explicit JuncGroup();
        
        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit JuncGroup( JuncGroup *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~JuncGroup();
        
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
        * get the pointer to the first junction reference in the group
        * @return   pointer to first reference
        */
        JuncRef* getFirstJuncRef();
        
        /**
        * calculate the respective data after the preparation stage 
        */
        virtual void calcPostPrepareData();
        
        /**
        * get the bounding box of all paths in the junction
        */
        Bbox* getBoundingBox();
        
        /**
        * get the center and the circumscribing radius of the bounding box
        * @param x  center x position
        * @param y  center y position
        * @param radius radius of the circle circumscribing the entire box in the x/y plane
        */
        void getCtrAndRadius( double & x, double & y, double & radius );

    private:
        /**
        * calculate the bounding box
        */
        void calcBoundingBox();

    private:
        /**
        * bounding box of all paths in the junction
        */
        Bbox *mBbox;
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_JUNC_GROUP_HH */
