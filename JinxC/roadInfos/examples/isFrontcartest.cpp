#include "OpenDRIVE.hh"
#include "OdrManager.hh"
#include "posInfo.hpp"


#include <iostream>
#include <cmath>
// #include <sys/time.h> //timeval
using namespace std;

int main( ) {
  cout << "front test" << endl;
  OpenDrive::OdrManager manager;
  // bool flag = true;
  std::string odrFile = "D:\\evaluate\\roadInfos\\data\\Germany_2018.xodr";
  getPosInfo posManager;
  std::cout<<"hmx"<<endl;
  bool ret = posManager.init(odrFile);
  std::cout<<"gx"<<endl;
  OpenDrive::Position* pos = manager.createPosition();
  manager.activatePosition(pos);


  int a=-1;
  bool b=a;
  if (b)
  {
    std::cout <<"b"<< b << std::endl;
  }


  // timeval start, end;
  // gettimeofday(&start, NULL);
  // Point p1(3220.94, 1355.46, 0);
  // Point p1;
  // Point p2;
  // p1=(3527.001, -2999.897, 0);
  // p2=(3175.825, -2998.515, 0);
  // // float heading1 = 45;
  // float heading2 = 45;

  double p1x=1023.46;
  double p1y=-8.1;
  double p1z=0;
  std::cout<<"p1:("<<p1x<<","<<p1y<<","<<p1z<<")"<<endl;

  double p2x=1023.89;
  double p2y=-91.35;
  double p2z=0;
  std::cout<<"p2:("<<p2x<<","<<p2y<<","<<p2z<<")"<<endl;

  float pi=4*atan(1);

  // std::cout<<"p2:"<<p2<<endl;
  float * length= new float;
  bool isfrontcarFlag = posManager.isFrontCar(Point(p1x,p1y,p1z),Point(p2x,p2y,p2z),pi/4,pi/4);
  std::cout<<"p1:("<<p1x<<","<<p1y<<","<<p1z<<")"<<endl;
  std::cout<<"p2:("<<p2x<<","<<p2y<<","<<p2z<<")"<<endl;
  std::cout<<"length:"<<*length<<endl;
  std::cout<<"isfrontcarFlag:"<<isfrontcarFlag<<endl;
  // vector<struct> vec = posManager.Preprocess1(Point (5979, -2815, 0) ,45);
  // for(vector<struct>::iterator it=sequence.begin();it!=sequence.end();it++){
  //       cout<<*it<<endl;
  //   }
  // int* roadconnectid = posManager.Preprocess(Point (5979, -2815, 0) ,45);
  // cout << roadconnectid[0] << " " << roadconnectid[1] << "  "<<roadconnectid[2]<<endl;

  
  //bool a = posManager.isOnRoadMark(Point (2385.36, -7268.76, 0));
  //bool a = posManager.isOnRoadMark(Point (2385.36, -7268.84, 0));
  //bool a = posManager.isOnRoadMark(Point (2385.36, -7265.28, 0));
  // bool a = posManager.isOnRoadMark(Point (-101.95, -69.14, 0));
  // std::cout  <<a <<  std::endl;
  // std::cout << "is front car " <<  std::endl;
  // std::cout << "is front car " << isfrontcarFlag << std::endl;
  return 0;
}