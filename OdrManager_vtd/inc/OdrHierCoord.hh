
#ifndef _OPENDRIVE_HIERCOORD_HH
#define _OPENDRIVE_HIERCOORD_HH
#include <vector>
namespace OpenDrive
{
  class Node;
  class HierCoord
  {
  private:
    typedef std::vector<Node *> NodeVec;
    NodeVec mNodeAccessVec;
    NodeVec mNodeAccessVecBackup;
    bool mLocked;
  public
      :
    explicit HierCoord();
    virtual ~HierCoord();
    void operator=(const HierCoord &Nf2Ao);
    void init();
    void print() const;
    bool keepNode(Node *node);
    bool clearNode(const unsigned int type);
    Node *findNode(const unsigned int type);
    bool registerNode(Node
                          *node,
                      const unsigned int type);
    void lock();
    void unlock();
    void backup();
    void restore();
  private:
    void checkSize(const unsigned int type);
  };
} // namespace OpenDrive
#endif
