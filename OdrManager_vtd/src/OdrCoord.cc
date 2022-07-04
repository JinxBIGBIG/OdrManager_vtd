
#include <iostream>
#include <math.h>
#include "OdrCoord.hh"
namespace OpenDrive{double Coord::getDist(const Coord&NSLYe,const Coord&rzkwu){
Coord pREJX=NSLYe;pREJX-=rzkwu;return pREJX.getValue();}double Coord::getDist2d(
const Coord&NSLYe,const Coord&rzkwu){Coord pREJX=NSLYe;pREJX-=rzkwu;pREJX.setZ(
0.0);return pREJX.getValue();}const double Coord::Rad2Deg=57.2957795132;const 
double Coord::Deg2Rad=0.0174532925199;Coord::Coord():mX(0.0),mY(0.0),mZ(0.0),mH(
0.0),mP(0.0),mR(0.0){}Coord::Coord(const double&x,const double&y,const double&
hkK5C,const double&aOzQT,const double&p,const double&yJG47):mX(x),mY(y),mZ(hkK5C
),mH(aOzQT),mP(p),mR(yJG47){}Coord::~Coord(){}void Coord::operator=(const Coord&
Nf2Ao){return set(Nf2Ao.getX(),Nf2Ao.getY(),Nf2Ao.getZ(),Nf2Ao.getH(),Nf2Ao.getP
(),Nf2Ao.getR());}Coord Coord::operator*(const double&Nf2Ao){return Coord(mX*
Nf2Ao,mY*Nf2Ao,mZ*Nf2Ao,mH*Nf2Ao,mP*Nf2Ao,mR*Nf2Ao);}Coord Coord::operator+(
const Coord&Nf2Ao){return Coord(mX+Nf2Ao.getX(),mY+Nf2Ao.getY(),mZ+Nf2Ao.getZ(),
mH+Nf2Ao.getH(),mP+Nf2Ao.getP(),mR+Nf2Ao.getR());}Coord Coord::operator-(const 
Coord&Nf2Ao){return Coord(mX-Nf2Ao.getX(),mY-Nf2Ao.getY(),mZ-Nf2Ao.getZ(),mH-
Nf2Ao.getH(),mP-Nf2Ao.getP(),mR-Nf2Ao.getR());}void Coord::operator-=(const 
Coord&Nf2Ao){mX-=Nf2Ao.getX();mY-=Nf2Ao.getY();mZ-=Nf2Ao.getZ();mH-=Nf2Ao.getH()
;mP-=Nf2Ao.getP();mR-=Nf2Ao.getR();}void Coord::operator+=(const Coord&Nf2Ao){mX
+=Nf2Ao.getX();mY+=Nf2Ao.getY();mZ+=Nf2Ao.getZ();mH+=Nf2Ao.getH();mP+=Nf2Ao.getP
();mR+=Nf2Ao.getR();}const double&Coord::getX()const{return mX;}const double&
Coord::getY()const{return mY;}const double&Coord::getZ()const{return mZ;}const 
double&Coord::getH()const{return mH;}const double&Coord::getP()const{return mP;}
const double&Coord::getR()const{return mR;}void Coord::set(const double&x,const 
double&y,const double&hkK5C,const double&aOzQT,const double&p,const double&yJG47
){mX=x;mY=y;mZ=hkK5C;mH=aOzQT;mP=p;mR=yJG47;}void Coord::setX(const double&value
){mX=value;}void Coord::setY(const double&value){mY=value;}void Coord::setZ(
const double&value){mZ=value;}void Coord::setH(const double&value){mH=value;}
void Coord::setP(const double&value){mP=value;}void Coord::setR(const double&
value){mR=value;}void Coord::init(){mX=0.0;mY=0.0;mZ=0.0;mH=0.0;mP=0.0;mR=0.0;}
void Coord::print()const{std::cerr<<
"\x43\x6f\x6f\x72\x64\x3a\x20\x78\x20\x3d\x20"<<mX<<"\x2c\x20\x79\x20\x3d\x20"<<
mY<<"\x2c\x20\x7a\x20\x3d\x20"<<mZ<<"\x2c\x20\x68\x20\x3d\x20"<<mH<<
"\x2c\x20\x70\x20\x3d\x20"<<mP<<"\x2c\x20\x72\x20\x3d\x20"<<mR<<std::endl;}
double Coord::getValue(){return sqrt(mX*mX+mY*mY+mZ*mZ);}}
