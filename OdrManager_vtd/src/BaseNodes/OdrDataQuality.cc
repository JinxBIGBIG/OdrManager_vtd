
#include "OdrDataQuality.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
namespace OpenDrive{DataQuality::DataQuality():Node(
"\x44\x61\x74\x61\x51\x75\x61\x6c\x69\x74\x79"){mOpcode=ODR_OPCODE_DATA_QUALITY;
}DataQuality::DataQuality(DataQuality*Nf2Ao):Node(Nf2Ao){}DataQuality::~
DataQuality(){}void DataQuality::printData()const{}bool DataQuality::read(
ReaderXML*){return true;}Node*DataQuality::getCopy(bool Mupxf){Node*dzamm=new 
DataQuality(this);if(Mupxf)deepCopy(dzamm);return dzamm;}}
