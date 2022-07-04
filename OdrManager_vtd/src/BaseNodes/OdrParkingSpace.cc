
#include "OdrParkingSpace.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{ParkingSpace::ParkingSpace():Node(
"\x50\x61\x72\x6b\x69\x6e\x67\x53\x70\x61\x63\x65"){mOpcode=
ODR_OPCODE_PARKING_SPACE;mLevel=2;}ParkingSpace::ParkingSpace(ParkingSpace*Nf2Ao
):Node(Nf2Ao){mAccess=Nf2Ao->mAccess;}ParkingSpace::~ParkingSpace(){}void 
ParkingSpace::printData()const{fprintf(stderr,
"\x20\x20\x20\x20\x20\x20\x20\x20\x41\x63\x63\x65\x73\x73\x3a\x20\x20\x20\x20\x25\x73" "\n"
,mAccess.c_str());}bool ParkingSpace::read(ReaderXML*F3vnM){mAccess=F3vnM->
getString("\x61\x63\x63\x65\x73\x73");return true;}Node*ParkingSpace::getCopy(
bool Mupxf){Node*dzamm=new ParkingSpace(this);if(Mupxf)deepCopy(dzamm);return 
dzamm;}}
