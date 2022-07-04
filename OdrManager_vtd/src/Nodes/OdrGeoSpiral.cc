
#include "OdrGeoHeader.hh"
#include "OdrGeoSpiral.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
namespace OpenDrive
{
  GeoSpiral::GeoSpiral() : GeoSpiralOdr()
  {
    mOpcode = ODR_OPCODE_GEO_SPIRAL;
    mLevel = 2;
  }
  GeoSpiral::~GeoSpiral() {}
} // namespace OpenDrive
