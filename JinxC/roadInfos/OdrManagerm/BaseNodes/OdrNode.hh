/* ===================================================
 *  file:       OdrNode.hh
 * ---------------------------------------------------
 *  purpose:	base class for holding OpenDRIVE
 *              information
 * ---------------------------------------------------
 *  first edit:	01.11.2005 by M. Dupuis @ VIRES GmbH
 * ===================================================
 */
#ifndef _OPENDRIVE_NODE_HH
#define _OPENDRIVE_NODE_HH

// ====== INCLUSIONS ======
#include <string>
#include <map>
#include "OpenDRIVE.hh"

namespace OpenDrive
{
    
class Reader;
class ReaderXML;
class ParserCallback;
    
class Node
{
    private:
    
        static const unsigned int sFlagClone = 0x01000000;
        
    private:
        /**
        * hold the name of the node (type identifier)
        */
        std::string mIdentifier;
        
        /**
        * left sibling
        */
        Node* mLeft;
        
        /**
        * right sibling
        */
        Node* mRight;
        
        /**
        * parent
        */
        Node* mParent;
        
        /**
        * hold a map of children per type
        */
        typedef std::map < unsigned int, Node* > NodeMap;
        NodeMap mChildMap;
        
        /**
        * hold a map of last children per type; accelerates loading of large databases
        * @version 1.5.3
        */
        NodeMap mLastChildMap;
        
        /**
        * pointer to arbitrary (user) data
        */
        void *mData;
        
        /**
        * set any CDATA that may be part of the node
        */
        std::string mCDATA;
        
    protected:
        /**
        * hold arbitrary debug information
        */
        unsigned int mDebugInfo;
        
        /**
        * hold info about source line no.
        */
        unsigned int mLineNo;

        /**
        * type ID (opcode) of the node
        */
        unsigned int mOpcode;
        
        /**
        * hierarchy level of this record
        */
        unsigned int mLevel;

    public:
        /**
        * constructor
        *   @param  name    name under which data is managed
        */
        explicit Node( const std::string & name );
        
        /**
        * copy constructor
        *  @param  rhs source node which is to be copied
        */
        explicit Node( Node *rhs );

        /** 
        * Destroy the class. 
        */
        virtual ~Node();
        
        /**
        * member access function
        *   @return name of the node
        */
        const std::string & getName() const;
        
        /**
        * member access function
        *   @return type of the node
        */
        unsigned int getOpcode() const;
        
        /**
        * member access function
        * @return level of the node
        */
        unsigned int getLevel() const;
        
        /**
        * member access function
        * @return left sibling of the node
        */
        Node* getLeft();
        const Node* getLeft() const;
        
        /**
        * member access function
        * @return right sibling of the node
        */
        Node* getRight();
        const Node* getRight() const;
        
        /**
        * member access function
        * @return parent of the node
        */
        Node* getParent();
        const Node* getParent() const;
        
        /**
        * get the number of child types under the node
        * @return number of child types under the node
        */
        unsigned int getNumChildTypes() const;
        
        /**
        * member access function
        * @param  type the type of child to be returned
        * @return child of the node
        */
        Node* getChild( unsigned int type );
        
        /**
        * member access function
        * @param  type the type of child to be returned
        * @return last child of the node of a given type
        */
        Node* getLastChild( unsigned int type );
        
        /**
        * member access function, returns the first child node
        * @return child of the node
        */
        Node* getChild();
        
        /**
        * get the child at a given type index
        * @param  index  the index of the type entry
        * @return child of the node
        */
        Node* getChildAtTypeIdx( unsigned int index );
        
        /**
        * add a child to the node
        * @param child   child node to be added
        * @param first   insert child as first child
        */
        void addChild( Node* child, bool first = false );
        
        /**
        * add a left sibling to the node
        * @param sibling   sibling node to be added
        */
        void addLeftSibling( Node* sibling );
        
        /**
        * add a right sibling to the node
        * @param sibling   sibling node to be added
        */
        void addRightSibling( Node* sibling );
        
        /**
        * set left sibling of the node
        * @param sibling   sibling node to be registered
        */
        void setLeft( Node* sibling );
        
        /**
        * set right sibling of the node
        * @param sibling   sibling node to be registered
        */
        void setRight( Node* sibling );
        
        /**
        * set the parent node to given node
        * @param parent parent node to be registered
        */
        void setParent( Node* parent );
        
        /**
        * move the node one position to the right
        * @return true if successful
        */
        bool shiftRight();
        
        /**
        * print the node
        * @param deep follow child nodes when printing
        * @param siblings print the siblings
        */
        void print( bool deep = true, bool siblings = true );
        
        /**
        * print the data contents of the node
        */
        virtual void printData() const;
        
        /**
        * read the contents of the node from a prepared attribute list
        * @param reader     the reader to be used for reading the data
        * @return failure/success
        */
        virtual bool read( ReaderXML* reader );
        
        /**
        * parse the node's data and call the corrsponding callback method
        * @param callback   the method to call upon parsing the data
        * @param deep       follow child nodes when parsing
        */
        void parse( ParserCallback* callback, bool deep = true );
         
        /**
        * trigger preparation of node for subsequent operations
        * called once immediately after reading a file
        * @param deep       follow child nodes when parsing
        */
        void prepare( bool deep = true );
         
        /**
        * calculate the respective data during the preparation stage 
        * called from the prepare() routine, may be overwritten in
        * the respective nodes
        */
        virtual void calcPrepareData();
        
        /**
        * apply inertial transformation operation on a single node
        * @param dx inertial x transformation (shift) [m]
        * @param dy inertial y transformation (shift) [m]
        * @param dz inertial z transformation (shift) [m]
        * @param dh inertial heading transformation (rotation) [rad]
        * @version 1.5.0
        */
        virtual void applyTransformation( const double & dx, const double & dy, const double &dz, const double & dh );

        /**
        * calculate the respective data after the preparation stage 
        * called from the prepare() routine, may be overwritten in
        * the respective nodes
        */
        virtual void calcPostPrepareData();

        /**
        * isolate a node from its parent and siblings
        */
        void isolate();
        
        /**
        * set the user data pointer
        * @param data   new data pointer
        */
        void setData( void* data );
        
        /**
        * get the user data pointer
        * @return pointer to user data
        */
        void* getData();
        const void* getData() const;
        
        /**
        * get the debug information
        * @return debug info variable
        */
        unsigned int getDebugInfo() const;
        
        /**
        * get the line number from the debug info
        * @return source line number
        */
        unsigned int getLineNo() const;
        
        /**
        * get the number of the read sequence during which the node was created
        * @return number of read sequence
        */
        unsigned int getSequenceNo() const;
        
        /**
        * check whether a node is a clone of an existing node
        * @return true if node has been cloned
        */
        bool isCloned() const;
        
        /**
        * set the number of the line at which node was located in the source file
        * and set the sequence number of the read operation when ist was read
        * @param lineNo source line number
        * @param seq    sequence number of the read operation
        */
        void setLineNo( const unsigned int & lineNo, const unsigned int & seq = 0 );
        
        /**
        * get the s co-ordinate of the node (if applicable),
        * required for general sort function 
        * (only for nodes which provide s value, of course)
        * @return s value or -1.0 if not applicable.
        */
        virtual const double & getS() const;
        
        /**
        * get a copy of the node in base class type
        * @param deep   set to true for a deep copy
        */
        virtual Node* getCopy( bool deep = false );
        
        /**
        * do a deep copy of node contents
        * @param parent pointer to the parent to which children shall be attached
        */
        void deepCopy( Node* parent );
        
        /**
        * set the level of the node
        * @param level  the level that is to be set
        */
        void setLevel( const unsigned int & level );
        
        /**
        * get the CDATA of a node
        * @return node's CDATA or empty string
        */
        const std::string & getCDATA() const;
        
        /**
        * set CDATA of a node
        * @param cdata CDATA that is to be remembered
        */
        void setCDATA( const std::string & cdata );
        
        /**
        * transform a node, siblings and - optionally - children
        * @param dx inertial x transformation (shift) [m]
        * @param dy inertial y transformation (shift) [m]
        * @param dz inertial z transformation (shift) [m]
        * @param dh inertial heading transformation (rotation) [rad]
        * @param deep follow child nodes when parsing
        * @version 1.5.0
        */
        virtual void transform( const double & dx = 0.0, const double & dy = 0.0, const double &dz = 0.0, const double & dh = 0.0, bool deep = true );
        
        /**
        * rotate a given x/y point around a center
        * @param x point's x co-ordinate [m]
        * @param y point's y co-ordinate [m]   
        * @param ctrX rotation center's x co-ordinate [m]
        * @param ctrY rotation center's y co-ordinate [m]   
        * @param angle rotation angle [rad]
        * @version 1.5.0
        */
        virtual void rotatePoint( double & x, double & y, const double &ctrX, const double & ctrY, const double & angle );

    private:
        /**
        * set the child node to given node
        * @param child child node to be registered
        * @param type  type of the child node
        */
        void setChild( Node* child, unsigned int type );
};
} // namespace OpenDRIVE

#endif /* _OPENDRIVE_NODE_HH */
