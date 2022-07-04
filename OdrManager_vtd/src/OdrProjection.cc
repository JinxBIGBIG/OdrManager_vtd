
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "OdrProjection.hh"
#include "WKTParser.hh"
#ifdef USE_PROJ_4    
#include "proj_api.h"
#endif
namespace OpenDrive{const double Projection::cDeg2Rad=M_PI/180.0;const std::
string Projection::cUTMString=std::string(
"\x2b\x70\x72\x6f\x6a\x3d\x75\x74\x6d\x20\x2b\x7a\x6f\x6e\x65\x3d\x33\x32\x20\x2b\x65\x6c\x6c\x70\x73\x3d\x57\x47\x53\x38\x34\x20\x2b\x64\x61\x74\x75\x6d\x3d\x57\x47\x53\x38\x34\x20\x2b\x75\x6e\x69\x74\x73\x3d\x6d\x20\x2b\x6e\x6f\x5f\x64\x65\x66\x73"
);const std::string Projection::cWGSString=std::string(
"\x2b\x70\x72\x6f\x6a\x3d\x6c\x61\x74\x6c\x6f\x6e\x67\x20\x2b\x64\x61\x74\x75\x6d\x3d\x57\x47\x53\x38\x34"
);Projection::Projection():mOGCWKTString(""),mPj_latlong(0),mPj_utm(0){
mProj4StringInertial=cUTMString;mProj4StringGeo=cWGSString;init();}Projection::~
Projection(){
#ifdef USE_PROJ_4    
if(mPj_utm)pj_free(mPj_utm);if(mPj_latlong)pj_free(mPj_latlong);
#endif
}void Projection::setOGCWKT(const std::string&IuLIB){static bool Y3vvh=false;if(
IuLIB.find("\x2b\x70\x72\x6f\x6a\x3d")!=std::string::npos){if(Y3vvh)fprintf(
stderr,
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x73\x65\x74\x4f\x47\x43\x57\x4b\x54\x3a\x20\x73\x65\x74\x74\x69\x6e\x67\x20\x70\x72\x6f\x6a\x34\x20\x3c\x25\x73\x3e" "\n"
,IuLIB.c_str());setProj4Inertial(IuLIB);return;}mOGCWKTString=IuLIB;WKTParser 
C5Wm8(mOGCWKTString);C5Wm8.parse();if(Y3vvh)C5Wm8.print();OpenDrive::WKTNode*
node=C5Wm8.getRootNode();bool H9VP_=false;std::string j612A="\x30\x2e\x30";std::
string wK9Dq="\x30\x2e\x30";std::string nf5ah="\x30\x2e\x30";std::string WHMPP=
"\x30\x2e\x30";std::string CP2qK="\x31\x2e\x30";while(node){if(node->getName()==
"\x50\x52\x4f\x4a\x45\x43\x54\x49\x4f\x4e"){if(node->getAttribute(0)==
"\"" "\x54\x72\x61\x6e\x73\x76\x65\x72\x73\x65\x5f\x4d\x65\x72\x63\x61\x74\x6f\x72" "\""
)H9VP_=true;}if(H9VP_&&(node->getName()=="\x50\x41\x52\x41\x4d\x45\x54\x45\x52")
){if(node->getAttribute(0)==
"\"" "\x6c\x61\x74\x69\x74\x75\x64\x65\x5f\x6f\x66\x5f\x6f\x72\x69\x67\x69\x6e" "\""
)j612A=node->getAttribute(1);if(node->getAttribute(0)==
"\"" "\x63\x65\x6e\x74\x72\x61\x6c\x5f\x6d\x65\x72\x69\x64\x69\x61\x6e" "\"")
wK9Dq=node->getAttribute(1);if(node->getAttribute(0)==
"\"" "\x73\x63\x61\x6c\x65\x5f\x66\x61\x63\x74\x6f\x72" "\"")CP2qK=node->
getAttribute(1);if(node->getAttribute(0)==
"\"" "\x66\x61\x6c\x73\x65\x5f\x65\x61\x73\x74\x69\x6e\x67" "\"")nf5ah=node->
getAttribute(1);if(node->getAttribute(0)==
"\"" "\x66\x61\x6c\x73\x65\x5f\x6e\x6f\x72\x74\x68\x69\x6e\x67" "\"")WHMPP=node
->getAttribute(1);}if(node->getChild())node=node->getChild();else if(node->
getSibling())node=node->getSibling();else{node=node->getParent();if(node)node=
node->getSibling();}}if(!H9VP_)return;std::string amoQ9=
"\x2b\x70\x72\x6f\x6a\x3d\x74\x6d\x65\x72\x63\x20\x2b\x6c\x6f\x6e\x5f\x30\x3d"+
wK9Dq+"\x20\x2b\x6c\x61\x74\x5f\x30\x3d"+j612A;amoQ9+=
"\x20\x2b\x65\x6c\x6c\x70\x73\x3d\x57\x47\x53\x38\x34";amoQ9+=
"\x20\x2b\x6b\x5f\x30\x3d"+CP2qK+"\x20\x2b\x78\x5f\x30\x3d"+nf5ah+
"\x20\x2b\x79\x5f\x30\x3d"+WHMPP;if(Y3vvh)fprintf(stderr,
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x73\x65\x74\x4f\x47\x43\x57\x4b\x54\x3a\x20\x73\x65\x74\x74\x69\x6e\x67\x20\x70\x72\x6f\x6a\x34\x20\x3c\x25\x73\x3e" "\n"
,amoQ9.c_str());setProj4Inertial(amoQ9);}void Projection::setProj4Inertial(const
 std::string&gBhbI){mProj4StringInertial=gBhbI;init();}void Projection::
setProj4Geo(const std::string&gBhbI){mProj4StringGeo=gBhbI;init();}void 
Projection::init(){static bool Y3vvh=false;if(Y3vvh){fprintf(stderr,
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x69\x6e\x69\x74\x3a\x20\x74\x68\x69\x73\x20\x3d\x20\x25\x70\x2c\x20\x67\x65\x6f\x20\x73\x74\x72\x69\x6e\x67\x20\x3d\x20\x3c\x25\x73\x3e" "\n"
,this,mProj4StringGeo.c_str());fprintf(stderr,
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x69\x6e\x69\x74\x3a\x20\x74\x68\x69\x73\x20\x3d\x20\x25\x70\x2c\x20\x69\x6e\x65\x72\x74\x69\x61\x6c\x20\x73\x74\x72\x69\x6e\x67\x20\x3d\x20\x3c\x25\x73\x3e" "\n"
,this,mProj4StringInertial.c_str());}
#ifdef USE_PROJ_4    
if(mPj_utm){pj_free(mPj_utm);mPj_utm=0;}if(mPj_latlong){pj_free(mPj_latlong);
mPj_latlong=0;}if(!(mPj_utm=(void*)pj_init_plus(mProj4StringInertial.c_str())))
fprintf(stderr,
"\x23\x20\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x69\x6e\x69\x74\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x20\x70\x72\x6f\x6a\x34\x20\x6c\x69\x62\x72\x61\x72\x79\x2c\x20\x49\x4e\x45\x52\x54\x49\x41\x4c\x20\x70\x61\x72\x74" "\n"
);if(!(mPj_latlong=(void*)pj_init_plus(mProj4StringGeo.c_str())))fprintf(stderr,
"\x23\x20\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x69\x6e\x69\x74\x3a\x20\x63\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x20\x70\x72\x6f\x6a\x34\x20\x6c\x69\x62\x72\x61\x72\x79\x2c\x20\x57\x47\x53\x38\x34\x20\x70\x61\x72\x74" "\n"
);
#endif
}int Projection::geo2inertial(const GeoCoord&V5tBk,Coord&Tlj7t){
#ifdef USE_PROJ_4    
double Y4isd=V5tBk.getLong();double iC17Q=V5tBk.getLat();double nzUBK=V5tBk.getZ
();Y4isd*=cDeg2Rad;iC17Q*=cDeg2Rad;int p=0;if(mPj_latlong&&mPj_utm)p=
pj_transform((projPJ*)mPj_latlong,(projPJ*)mPj_utm,1,1,&Y4isd,&iC17Q,&nzUBK);
Tlj7t.setX(Y4isd-mOriginInertial.getX());Tlj7t.setY(iC17Q-mOriginInertial.getY()
);Tlj7t.setZ(nzUBK-mOriginInertial.getZ());Tlj7t.setH(0.5*M_PI-V5tBk.getH());
Tlj7t.setP(V5tBk.getP());Tlj7t.setR(V5tBk.getR());return p;
#else
Tlj7t.setX(V5tBk.getLong());Tlj7t.setY(V5tBk.getLat());Tlj7t.setZ(V5tBk.getZ());
Tlj7t.setH(0.5*M_PI-V5tBk.getH());Tlj7t.setP(V5tBk.getP());Tlj7t.setR(V5tBk.getR
());return 0;
#endif
}int Projection::inertial2geo(const Coord&Tlj7t,GeoCoord&V5tBk){
#ifdef USE_PROJ_4    
double Y4isd=Tlj7t.getX()+mOriginInertial.getX();double iC17Q=Tlj7t.getY()+
mOriginInertial.getY();double nzUBK=Tlj7t.getZ()+mOriginInertial.getZ();int p=0;
if(mPj_utm&&mPj_latlong){p=pj_transform((projPJ*)mPj_utm,(projPJ*)mPj_latlong,1,
1,&Y4isd,&iC17Q,&nzUBK);Y4isd/=cDeg2Rad;iC17Q/=cDeg2Rad;}V5tBk.setLong(Y4isd);
V5tBk.setLat(iC17Q);V5tBk.setZ(nzUBK);V5tBk.setH(0.5*M_PI-Tlj7t.getH());V5tBk.
setP(Tlj7t.getP());V5tBk.setR(Tlj7t.getR());while(V5tBk.getH()<0.0)V5tBk.setH(
V5tBk.getH()+2.0*M_PI);while(V5tBk.getH()>(2.0*M_PI))V5tBk.setH(V5tBk.getH()-2.0
*M_PI);
#ifdef nfV52
if(1){struct jvI7O LowAd;int UbLns=pj_factors(data,(projPJ*)mPj_utm,0.,&LowAd);
if(!UbLns)fprintf(stderr,
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x3a\x69\x6e\x65\x72\x74\x69\x61\x6c\x32\x67\x65\x6f\x3a\x20\x63\x6f\x6e\x76\x65\x72\x67\x65\x6e\x63\x65\x20\x3d\x20\x25\x2e\x36\x6c\x66" "\n"
,LowAd.D5L0T);}
#endif    
return p;
#else
V5tBk.setLong(Tlj7t.getX()+mOriginInertial.getX());V5tBk.setLat(Tlj7t.getY()+
mOriginInertial.getY());V5tBk.setZ(Tlj7t.getZ()+mOriginInertial.getZ());V5tBk.
setH(0.5*M_PI-Tlj7t.getH());V5tBk.setP(Tlj7t.getP());V5tBk.setR(Tlj7t.getR());
while(V5tBk.getH()<0.0)V5tBk.setH(V5tBk.getH()+2.0*M_PI);while(V5tBk.getH()>(2.0
*M_PI))V5tBk.setH(V5tBk.getH()-2.0*M_PI);return 0;
#endif
}void Projection::print()const{std::cerr<<
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x20\x4f\x47\x43\x2d\x57\x4b\x54\x20\x3d\x20\x3c"
<<mOGCWKTString<<"\x3e"<<std::endl;std::cerr<<
"\x50\x72\x6f\x6a\x65\x63\x74\x69\x6f\x6e\x3a\x20\x50\x52\x4f\x4a\x34\x20\x49\x4e\x20\x20\x3d\x20\x3c"
<<mProj4StringInertial<<
"\x3e\x20\x50\x52\x4f\x4a\x34\x20\x47\x45\x4f\x20\x20\x3d\x20\x3c"<<
mProj4StringGeo<<"\x3e"<<std::endl;}void Projection::setOriginInertial(const 
Coord&offset){mOriginInertial=offset;}const Coord&Projection::getOriginInertial(
)const{return mOriginInertial;}}
