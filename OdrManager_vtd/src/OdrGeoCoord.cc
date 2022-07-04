
#include <iostream>
#include <math.h>
#include "OdrGeoCoord.hh"
namespace OpenDrive{const double GeoCoord::cDegPerM=360.0/(2*M_PI*6378000.0);
GeoCoord::GeoCoord():mLong(0.0),mLat(0.0),mZ(0.0),mH(0.0),mP(0.0),mR(0.0){}
GeoCoord::GeoCoord(const double&auc9o,const double&smK7U,const double&hkK5C,
const double&aOzQT,const double&p,const double&yJG47):mLong(auc9o),mLat(smK7U),
mZ(hkK5C),mH(aOzQT),mP(p),mR(yJG47){}GeoCoord::~GeoCoord(){}void GeoCoord::
operator=(const GeoCoord&Nf2Ao){return set(Nf2Ao.getLong(),Nf2Ao.getLat(),Nf2Ao.
getZ(),Nf2Ao.getH(),Nf2Ao.getP(),Nf2Ao.getR());}GeoCoord GeoCoord::operator*(
const double&Nf2Ao){return GeoCoord(mLong*Nf2Ao,mLat*Nf2Ao,mZ*Nf2Ao,mH*Nf2Ao,mP*
Nf2Ao,mR*Nf2Ao);}GeoCoord GeoCoord::operator+(const GeoCoord&Nf2Ao){return 
GeoCoord(mLong+Nf2Ao.getLong(),mLat+Nf2Ao.getLat(),mZ+Nf2Ao.getZ(),mH+Nf2Ao.getH
(),mP+Nf2Ao.getP(),mR+Nf2Ao.getR());}GeoCoord GeoCoord::operator-(const GeoCoord
&Nf2Ao){return GeoCoord(mLong-Nf2Ao.getLong(),mLat-Nf2Ao.getLat(),mZ-Nf2Ao.getZ(
),mH-Nf2Ao.getH(),mP-Nf2Ao.getP(),mR-Nf2Ao.getR());}void GeoCoord::operator-=(
const GeoCoord&Nf2Ao){mLong-=Nf2Ao.getLong();mLat-=Nf2Ao.getLat();mZ-=Nf2Ao.getZ
();mH-=Nf2Ao.getH();mP-=Nf2Ao.getP();mR-=Nf2Ao.getR();}void GeoCoord::operator+=
(const GeoCoord&Nf2Ao){mLong+=Nf2Ao.getLong();mLat+=Nf2Ao.getLat();mZ+=Nf2Ao.
getZ();mH+=Nf2Ao.getH();mP+=Nf2Ao.getP();mR+=Nf2Ao.getR();}const double&GeoCoord
::getLong()const{return mLong;}const double&GeoCoord::getLat()const{return mLat;
}const double&GeoCoord::getZ()const{return mZ;}const double&GeoCoord::getH()
const{return mH;}const double&GeoCoord::getP()const{return mP;}const double&
GeoCoord::getR()const{return mR;}void GeoCoord::set(const double&auc9o,const 
double&smK7U,const double&hkK5C,const double&aOzQT,const double&p,const double&
yJG47){mLong=auc9o;mLat=smK7U;mZ=hkK5C;mH=aOzQT;mP=p;mR=yJG47;}void GeoCoord::
setLong(const double&value){mLong=value;}void GeoCoord::setLat(const double&
value){mLat=value;}void GeoCoord::setZ(const double&value){mZ=value;}void 
GeoCoord::setH(const double&value){mH=value;}void GeoCoord::setP(const double&
value){mP=value;}void GeoCoord::setR(const double&value){mR=value;}void GeoCoord
::init(){mLong=0.0;mLat=0.0;mZ=0.0;mH=0.0;mP=0.0;mR=0.0;}void GeoCoord::print()
const{std::cerr<<
"\x47\x65\x6f\x43\x6f\x6f\x72\x64\x3a\x20\x6c\x61\x74\x20\x3d\x20"<<mLong<<
"\x2c\x20\x6c\x6f\x6e\x67\x20\x3d\x20"<<mLat<<"\x2c\x20\x7a\x20\x3d\x20"<<mZ<<
"\x2c\x20\x68\x20\x3d\x20"<<mH<<"\x2c\x20\x70\x20\x3d\x20"<<mP<<
"\x2c\x20\x72\x20\x3d\x20"<<mR<<std::endl;}}
