#include "TinyXml/tinyxml.h"
#include "TinyXml/tinystr.h"

#include <iostream>
using namespace std;
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
	TiXmlDocument *Document = newTiXmlDocument ();
  if(!Document->LoadFile(m_strXmlPath.c_str()))
  {
    cout << "无法加载xml文件！" << endl;
    cin.get();
    return false; 
  } 
  
  {
    TiXmlElement *RootElement = Document->RootElement();		//根目录
	  TiXmlElement *NextElement = RootElement->FirstChildElement();		//根目录下的第一个节点层
    int roadNumi=0;
    // while(NextElement!=NULL)		//判断有没有读完
    // {
    //   if(NextElement->ValueTStr() == "road")		//读到road节点
    //   {
    //     TiXmlElement *lengthElemeng = NextElement->FirstChildElement();//索引到length节点
    //     pNode->length = atof(lengthElemeng->GetText());
    //     TiXmlElement *idElemeng = lengthElemeng->NextSiblingElement();
    //     pNode->id = atof(idElemeng->GetText());
    //     cout << "Road ID is" << pNode->id << endl;
    //     cout << "Road length is" << pNode->length << endl;    
    //     roadNumi++;     
    //   }
    // }    
    cout << "Road Number is" << roadNumi << endl;
    roadNumi=0;
  }  
  // delete pNode;
	// delete Document;
  cout << "end" << endl;

	return true;
}

int main(int argc, char** argv) {
   cout << "Road ID test" << endl;
   ReadRoadNum(string ("D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr"));
   return 0;
}

