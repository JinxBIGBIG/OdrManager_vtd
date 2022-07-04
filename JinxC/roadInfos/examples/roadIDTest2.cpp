#include "tinyxml2.h"
 #include "OpenDRIVE.hh"
 #include "OdrManager.hh"
 #include "BaseNodes/OdrRoadHeader.hh"
#include "BaseNodes/OdrLane.hh"
#include "OdrReaderXML.hh"
#include <iostream>
using namespace std;
using namespace tinyxml2;
using namespace OpenDrive;
bool ReadRoadNum(string m_strXmlPath)
{
  struct Attribute
  {
    int length;
    int id;
  };
	Attribute *pNode = new Attribute;
	//读取xml文件中的参数值
	XMLDocument *Document = new XMLDocument();
  if(Document->LoadFile(m_strXmlPath.c_str()) != XML_SUCCESS)
  {
    cout << "failed to load xml file." << endl;
    return false; 
  }

  else 
  {
      //Document->Print();
      XMLElement *RootElement = Document->RootElement();		//根目录OpenDRIVE
	  XMLElement *RoadElement = RootElement->FirstChildElement("road");		//根目录下的第一个节点层road

      int roadNumi=0;
    
    for (RoadElement; RoadElement->NextSiblingElement() != NULL; RoadElement=RoadElement->NextSiblingElement())
    {
      if ((RoadElement->Attribute("id")!=NULL)&(RoadElement->Attribute("length")!=NULL))
      {
        roadNumi++; 
        //cout << "Road ID is " << RoadElement->Attribute("id") << endl;
        //cout << "Road length is " << RoadElement->Attribute("length") << endl;
      }
      
    }
    //  while(RoadElement!=NULL)		//判断有没有读完(读取母节点下的几个子节点)
    // {
    //   // cout << "Road Number is " << endl;
    //   //const XMLAttribute* attributeOfRoad = RoadElement->FirstAttribute();  //获得road的name
    //   cout << "Road ID is " << RoadElement->Attribute("id") << endl;
    //   cout << "Road length is " << RoadElement->Attribute("length") << endl;
    //   roadNumi++; 
    //   //  cout << "road " << endl;
    //   //  if(RoadElement->FirstChildElement("road"))		//读到road节点
    //   //  {
    //   //    cout << "road is " << endl;
    //   //    XMLElement *lengthElemeng = RoadElement->FirstChildElement();//索引到length节点
    //   //    pNode->length = atof(lengthElemeng->GetText());
    //   //    XMLElement *idElemeng = lengthElemeng->NextSiblingElement();
    //   //    pNode->id = atof(idElemeng->GetText());
    //   //    cout << "Road ID is" << pNode->id << endl;
    //   //    cout << "Road length is" << pNode->length << endl;    
    //   //    roadNumi++;     
    //   //  }
    // }    
    cout << "Road Number is" << roadNumi << endl;
    roadNumi = 0;
  }  
  // delete pNode;
	// delete Document;
  cout << "end" << endl;
	return true;
}

int main(int argc, char** argv) {
   cout << "Road ID test" << endl;
   ReadRoadNum("D:\\MCworkspace\\JinxC\\roadInfos\\data\\Germany_2018.xodr");
   return 0;
}

