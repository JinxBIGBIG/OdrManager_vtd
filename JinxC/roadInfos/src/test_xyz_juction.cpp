#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrNode.hh"
#include "BaseNodes/OdrJuncLink.hh"
#include "BaseNodes/OdrJuncLaneLink.hh"
#include "BaseNodes/OdrJuncHeader.hh"

#include <iostream>
#include <cmath>
using namespace std;

int main(int argc, char** argv) {
    OpenDrive::OdrManager manager;
    string filePath = "D:/map_analysis/xyz/evaluate/roadInfos/data/Germany_2018.xodr";
    bool flag = manager.loadFile(filePath);
    if (flag)
        cout << "load file successful." << endl;
    else
        cout << "load file wrong." << endl;
    
    OpenDrive::Position* pos = manager.createPosition();
    manager.activatePosition(pos);

    manager.setInertialPos(2834.63,-7114.21,0);
    bool result = manager.inertial2lane();

    if(result){
        OpenDrive::RoadHeader* header = manager.getRoadHeader();
        // cout << header->mJuncNo << endl;
        // cout << manager.getJunctionId() << endl;


        int tempNum = manager.getJunctionId();  // junction id of road
        if (tempNum!=-1)
        {
            OpenDrive::Node* tempNode = header->getJunction();  // pointer to the junction
            OpenDrive::JuncHeader *myJunction = reinterpret_cast<OpenDrive::JuncHeader *>(tempNode);
            OpenDrive::JuncLink *myJuncLink = reinterpret_cast<OpenDrive::JuncLink *>(myJunction->getFirstLink());
            cout << "incoming road: " << myJuncLink->mIncomingRoad << endl;
            cout << "connecting road: " << myJuncLink->mConnectingRoad << endl;   
        }
        else
        {
            cout << "the point dose not belong to junction. " << endl;
        }
        
    }

    
}