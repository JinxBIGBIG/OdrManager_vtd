#include <iostream>
using namespace std;

junction
        connection
//gx:20220420：用于判断地图中是否有特定的车道类型，如公交车道、机动车道等。
//输入：1、m_strXmlPath:xodr文件路径；
//2、laneType:需要判断的车道类型：1：公交车道:bus；2：机动车道:driving；3：非机动车道:biking；
//输出：0：xodr文件中无该车道类型；1：xodr文件中有该车道类型；
bool getPreprocess::ReadLaneType(XMLDocument *Document, int laneType)
{
    string stringLaneType;
    string stringAccessType;
    // 公交车道
    if (laneType==1)
    {
        stringLaneType="bus";
        stringAccessType="bus";
    }
        // 机动车道
    else if (laneType==2)
    {
        stringLaneType="driving";
    }
        // 非机动车道
    else if (laneType==3)
    {
        stringLaneType="biking";
    }
    XMLElement *RootElement = Document->RootElement();		//OpenDRIVE
    XMLElement *RoadElement = RootElement->FirstChildElement("road");		//road
    XMLElement *lanesElement = RoadElement->FirstChildElement("lanes");		//lanes
    XMLElement *laneSectionElement = lanesElement->FirstChildElement("laneSection");		//laneSection
    XMLElement *laneleftElement = laneSectionElement->FirstChildElement("left");		//left
    XMLElement *leftlaneElement = laneleftElement->FirstChildElement("lane");		//leftlane
    XMLElement *lanecenterElement = laneSectionElement->FirstChildElement("center");		//center
    XMLElement *centerlaneElement = lanecenterElement->FirstChildElement("lane");		//centerlane
    XMLElement *lanerightElement = laneSectionElement->FirstChildElement("right");		//right
    XMLElement *rightlaneElement = lanerightElement->FirstChildElement("lane");		//rightlane
    if ((RootElement==NULL)||(RoadElement==NULL)||(lanesElement==NULL)||(laneSectionElement==NULL))
    {
        return 0;
    }
    for (RoadElement; RoadElement != NULL; RoadElement=RoadElement->NextSiblingElement())
    {
        // std::cout<<"roadid:"<<RoadElement->Attribute("id")<<std::endl;
        if (RoadElement==NULL)
        {
            return 0;
        }
        XMLElement *lanesElement = RoadElement->FirstChildElement("lanes");		//lanes
        if (lanesElement==NULL)
        {
            continue;
        }
        XMLElement *laneSectionElement = lanesElement->FirstChildElement("laneSection");		//laneSection
        if (laneSectionElement==NULL)
        {
            continue;
        }
        for (laneSectionElement; laneSectionElement!= NULL; laneSectionElement=laneSectionElement->NextSiblingElement())
        {
            // cout << "laneSection is " <<laneSectionElement->Attribute("s") << endl;
            XMLElement *laneleftElement = laneSectionElement->FirstChildElement("left");		//left
            XMLElement *lanecenterElement = laneSectionElement->FirstChildElement("center");		//center
            XMLElement *lanerightElement = laneSectionElement->FirstChildElement("right");		//right
            bool leftlaneEnd=0;
            bool centerlaneEnd=0;
            if ((laneleftElement==NULL)&&(lanecenterElement==NULL)&&(lanerightElement==NULL))
            {
                break;
            }
            if (laneleftElement==NULL)
            {
                leftlaneEnd=1;
            }
            else
            {
                XMLElement *leftlaneElement = laneleftElement->FirstChildElement("lane");		//leftlane
                if (leftlaneElement!=NULL)
                {
                    for (leftlaneElement; leftlaneElement != NULL; leftlaneElement=leftlaneElement->NextSiblingElement())
                    {
                        // std::cout<<"leftlane:"<<leftlaneElement->Attribute("id")<<std::endl;
                        if (leftlaneElement==NULL)
                        {
                            leftlaneEnd=1;
                            break;
                        }
                        else if ((leftlaneElement->Attribute("type")!=NULL))
                        {
                            if (leftlaneElement->Attribute("type")==stringLaneType)
                            {
                                return 1;
                            }
                        }
                        //if lane access is bus
                        XMLElement *leftlaneAccessElement = leftlaneElement->FirstChildElement("access");		//centerlaneAccess
                        if (leftlaneAccessElement!=NULL)
                        {
                            if (leftlaneAccessElement->Attribute("restriction")!=NULL)
                            {
                                if (leftlaneAccessElement->Attribute("restriction")==stringAccessType)
                                {
                                    return 1;
                                }
                            }
                        }
                    }
                    leftlaneEnd=1;
                }
            }
            if(lanecenterElement==NULL)
            {
                centerlaneEnd=1;
            }
            else
            {
                XMLElement *centerlaneElement = lanecenterElement->FirstChildElement("lane");		//centerlane
                if ((centerlaneElement!=NULL)&&(leftlaneEnd==1))
                {
                    for (centerlaneElement; centerlaneElement != NULL; centerlaneElement=centerlaneElement->NextSiblingElement())
                    {
                        if (centerlaneElement==NULL)
                        {
                            centerlaneEnd=1;
                            break;
                        }
                            //if lane type is bus
                        else if ((centerlaneElement->Attribute("type")!=NULL))
                        {
                            if (centerlaneElement->Attribute("type")==stringLaneType)
                            {
                                return 1;
                            }
                        }
                        //if lane access is bus
                        // std::cout<<"centerlaneElement:"<<centerlaneElement->Attribute("id")<<std::endl;
                        XMLElement *centerlaneAccessElement = centerlaneElement->FirstChildElement("access");		//centerlaneAccess
                        if (centerlaneAccessElement!=NULL)
                        {
                            if (centerlaneAccessElement->Attribute("restriction")!=NULL)
                            {
                                if (centerlaneAccessElement->Attribute("restriction")==stringAccessType)
                                {
                                    return 1;
                                }
                            }
                        }
                    }
                    centerlaneEnd=1;
                }
            }
            if (lanerightElement!=NULL)
            {
                XMLElement *rightlaneElement = lanerightElement->FirstChildElement("lane");		//rightlane
                if ((rightlaneElement!=NULL)&&(centerlaneEnd==1))
                {
                    for (rightlaneElement; rightlaneElement != NULL; rightlaneElement=rightlaneElement->NextSiblingElement())
                    {

                        // std::cout<<"rightlaneElement:"<<rightlaneElement->Attribute("id")<<std::endl;
                        if (rightlaneElement==NULL)
                        {
                            break;
                        }
                        else if ((rightlaneElement->Attribute("type")!=NULL))
                        {
                            if (rightlaneElement->Attribute("type")==stringLaneType)
                            {
                                return 1;
                            }
                        }
                        //if lane access is bus
                        XMLElement *rightlaneAccessElement = rightlaneElement->FirstChildElement("access");		//centerlaneAccess
                        if (rightlaneAccessElement!=NULL)
                        {
                            if (rightlaneAccessElement->Attribute("restriction")!=NULL)
                            {
                                if (rightlaneAccessElement->Attribute("restriction")==stringAccessType)
                                {
                                    return 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int main() {
  cout << "hello world." << endl;
    laneOffset
  return 0;
}