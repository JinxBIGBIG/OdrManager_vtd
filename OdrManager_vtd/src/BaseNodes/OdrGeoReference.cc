
#include "OdrGeoReference.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
namespace OpenDrive{GeoReference::GeoReference():Node(
"\x47\x65\x6f\x52\x65\x66\x65\x72\x65\x6e\x63\x65"){mOpcode=
ODR_OPCODE_GEO_REFERENCE;mLevel=1;}GeoReference::GeoReference(GeoReference*Nf2Ao
):Node(Nf2Ao){mDataString=Nf2Ao->mDataString;}GeoReference::~GeoReference(){}
void GeoReference::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x64\x61\x74\x61\x53\x74\x72\x69\x6e\x67\x3a\x20\x25\x73" "\n",
mDataString.c_str());}bool GeoReference::read(ReaderXML*){return true;}Node*
GeoReference::getCopy(bool Mupxf){Node*dzamm=new GeoReference(this);if(Mupxf)
deepCopy(dzamm);return dzamm;}}
