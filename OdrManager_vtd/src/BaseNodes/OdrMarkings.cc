
#include "OdrMarkings.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
namespace OpenDrive{Markings::Markings():Node("\x4d\x61\x72\x6b\x69\x6e\x67\x73"
){mOpcode=ODR_OPCODE_MARKINGS;mLevel=2;}Markings::Markings(Markings*Nf2Ao):Node(
Nf2Ao){}Markings::~Markings(){}void Markings::printData()const{}bool Markings::
read(ReaderXML*){return true;}Node*Markings::getCopy(bool Mupxf){Node*dzamm=new 
Markings(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
