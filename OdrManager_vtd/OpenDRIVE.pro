# get the version of this library from file
VERSION = $$cat( CURRENT_VERSION )

# version 1.5.5:  02.08.2019 - fixed successive loading of databases
# version 1.5.4:  26.07.2019 - performance optimization for large databases, ticket #10778
# version 1.5.3:  22.07.2019 - debugging on-site at customer
# version 1.5.2:  21.07.2019 - correcting inertial2lane calculation, ticket #9799, #10531
# version 1.5.1:  19.07.2019 - adding loader option LOADER_INHIBIT_CURVATURE_APPROXIMATION, ticket #7449
# version 1.5.0:  30.05.2019 - started implementation of OpenDRIVE 1.5
# version 1.4.39: 26.04.2019 - interpreting road speed node, ticket #10054
# version 1.4.38: 06.03.2019 - removed old definition EnRoadMark, ticket #9534
# version 1.4.36: 12.12.2018 - activating support for preliminary ODR 1.5 feature, ticket #8668
# version 1.4.35: 10.12.2018 - for round objects, sizeX and sizeY are both set to double the radius, ticket #8917
# version 1.4.34: 10.12.2018 - preliminary ODR 1.5 features if "ODR_1_5_PREL" is defined, ticket #8668
# version 1.4.33: 26.11.2018 - fixed inertial2lane issue, ticket #8403
# version 1.4.32: 10.11.2018 - got rid of Win32Defines, ticket #7709
#                            - added C-support, ticket #8280
#                            - removed all but one _WIN32 check
#                            - update to OpenCRG 1.1.2
# version 1.4.31: 19.08.2018 - working on inertial2laneList, ticket #7709
#                            - fixed calculation of vertical curvature, report by colleague
# version 1.4.30: 05.08.2018 - fixed performance problems, ticket #8084
# version 1.4.29: 04.07.2018 - lanes are sorted within a lane section before border lanes are added, ticket #7668
#                            - lane sections are identified from given lane position using also the singleSide information, ticket #7709
#                            - lane sections with mixed width and border entries for lanes are handled, ticket #7281
#                            - added loader option LOADER_IGNORE_SURFACE, ticket #7918
# version 1.4.28: 12.06.2018 - removing segmentation fault at end of execution if roadData has been copied
# version 1.4.27: 09.05.2018 - bumpy road elevation from inertial to z for super-elevated roads composed of line segments 
#                            - debug version only for ticket #7449
# version 1.4.26: 21.04.2018 (- parking lanes are considered "driveable", bugreport via e-mail) temporarily disabled 
#                            - bugreport according to ticket #7302 fixed (data races)
#                            - supporting ADD_BORDER_LANES for singleSided lane sections, ticket #7123
#                            - fixed calculation of MinMax width of lanes, ticket #6534
# version 1.4.25: 24.02.2018 - multiple CRG files at a given location will be evaluated, ticket #6700
#                            - adding single-side lane sections, ticket #7123
# version 1.4.24: 04.01.2018 - working on lane border evaluation, ticket #6535
# version 1.4.23: 19.08.2017 - added <rule> below <lane>, ticket #6306
#                            - some modifications for compilation on Windows platforms, ticket #6300
# version 1.4.22: 15.08.2017 - fixed interpretation of signal value "no limit", ticket #5646
#                            - fixed heading calculation for "odd" databases, ticket #5834
# version 1.4.21: 28.02.2017 - fixed retrieval of surface code as a string, ticket #5346
#                            - added functionality to represent elevation in a junction by an OpenCRG file only (starting to work on Odr 1.5)
#                            - fixed length entry for resolved continuous objects, ticket #5941
# version 1.4.20: 26.02.2017 - bugreport by TC (paramPoly3 interpretation for "odd" definitions), ticket #5690
#                            - using memory for inertial2path() now, ticket #5555
#                            - fixed return value for failed adding of waypoints to a path, ticket #5426
#                            - adding curvature from laneOffset to curvature of track, ticket #5429
#                            - CRG file may now be given relative to OpenDRIVE file, ticket #5226
#                            - fixed handling of deltas upon conversion between track and inertial positions, ticket #4631
#                            - fixed interpretation of mDir in Surface bead, ticket #5693
#                            - fixed superposition of superelevation and lateral shape, ticket #5428                                
# version 1.4.19: 02.02.2017 - improved splitting of junctions, ticket #5193
#                            - added ObjectReference node, ticket #4835
#                            - fixed resolving of repeated objects, ticket #5631
#                            - added ObjectMaterial node below the object level, ticket #5600        
#                            - fixed use of ODR_DIRECTION_NONE for signals and objects               
# version 1.4.18: 11.01.2017 - converting heading value between GEO and INERTIAL co-ordinates; this had been ignored ever since
#                            - fixed calculation of curvature on line segments
# version 1.4.17: 07.10.2016 - fixed interpolation of line heading again, ticket #5232
# version 1.4.16: 23.09.2016 - fixed interpolation of line heading, ticket #5153
# version 1.4.15: 19.08.2016 - fixed lateral shape, ticket #5010
# version 1.4.14: 26.07.2016 - fixed a bug in copying paramPoly3 nodes, ticket #4847
# version 1.4.13: 22.07.2016 - fixed linear interpolation between successive lateral profile definitions
# version 1.4.12: 21.07.2016 - fixed length lookup table for parametric cubic polynomials
# version 1.4.11: 18.07.2016 - fixed calculation of lane heading, see ticket #4875
# version 1.4.10: 26.06.2016 - accepting wrong tag "<laneValidity>" for backward compatibility purposes, see ticket #4775
# version 1.4.9:  09.06.2016 - patched print routines in path, see ticket #4857, no new revision number
#                 23.05.2016 - added "isOptimumSolution" to distinguish between best and second-but-best result of inertial2lane()
# version 1.4.8:  22.05.2016 - removed OdrGenericNode from public include list, see today's bugreport by EFS via e-mail
#                            - made warning message in GeoParamPoly depending on sine value of angular difference, not on difference itself
#                            - split paths in junctions are linked correctly again, ticket #4367
#                            - updated to OpenCRG 1.1.1, ticket #3911
#                            - calculating pseudo curvature on tracks consisting of linear primitives only, tickets #1217 and #2164
#                            - added lane types "entry", "exit", "onRamp", "offRamp", "connectingRamp", ticket #4061
#                            - fixed calculation of lateral profile, ticket #4583
#                            - added tolerance in method "addLaneS()", ticket #4515
# version 1.4.7:  25.02.2016 - taking unit specification in LaneSpeed into account
# version 1.4.6:  11.11.2015 - introduced fixed z-tolerance for inertial2lane calculation, see define CHECK_Z_I2L
# version 1.4.5:  24.10.2015 - introduced Proj4 conversion
#                            - fixed lane offset calculation
#                            - implemented methods for roads identified by strings
#                               - bool addSurfaceCRG( const std::string & roadId, const std::string & file, const double & sStart,
#                                                     const double & sEnd, const unsigned char & orientation, const unsigned int & mode,
#                                                     const double & sOffset, const double & tOffset, const double & zOffset, const double & zScale,
#                                                     const double & hOffset, const unsigned short & purpose );
#                               - bool inertial2lane( const std::string& trackId );
#                               - void setTrackPos( const std::string & id, const double & s, const double & t = 0.0 );
#                               - void setLanePos( const std::string & trackId, const int & laneId, const double & s, const double & offset = 0.0 );
#                               - double getTrackLen( const std::string & trackId );
#                            - added method for adding signals at run-time
#                            - auto-detecting OGC vs. proj4 string as geo-reference
# version 1.4.4:  15.06.2015 - bugfixed railroad switches
# version 1.4.3:  31.05.2015 - bugfixed ParamPoly3 computation for "odd" definitions (au, av, bu, bv all non-zero)
# version 1.4.2:  11.05.2015 - introduced string IDs for basic features
#                            - introduced RoadNeighbor
# version 1.4.1:  14.04.2015 - added signal attributes
#                            - added parking space and parking space markings
# version 1.4.0:  03.03.2015 - added parametric cubic polynomial
#                            - added methods for evaluating CRG data on a patch
#                            - all files now have the pre-fix "Odr", the class names have not been touched

#VERSTRG = version-$$VERSION
#DEFINES += SWREVISION=\\\"$$VERSTRG\\\"

DEFINES += SWREVISION=\\\"$$VERSION\\\"
DEFINES += _USE_MATH_DEFINES

# Prevent platform specific behaviour
IGNORE_PLATFORM_LIBS = yes

INCLUDEPATH	+= ./inc

#
# let us do no debug version
#
#CONFIG += debug
#CONFIG -= warn_on
CONFIG -= qt
#CONFIG -= release
CONFIG += flags

#use proj4?
#CONFIG += use_proj4

#
# common compiler flags
# QMAKE_CFLAGS are used for C files
#
flags{
    !win32-msvc* {
        QMAKE_CXXFLAGS		+= -ansi -Wall -Wno-misleading-indentation -fsigned-char
        QMAKE_CFLAGS		+= -Wall -Wstrict-prototypes
        QMAKE_CC            = g++
        # for Linux and MinGW
        DEFINES += LINUX
    }
    win32-msvc* {
        QMAKE_CFLAGS += /TP
    }
}

#
# certainly we are using linux here
#
DEFINES += LINUX

#
# commonly used libraries
#
!win32-msvc* {
    #
    # commonly used libraries
    #
    LIBS	    += -Wl,-rpath ..
}

#
# Temporary objects
#
unix {
  OBJECTS_DIR = obj
}

# Proj4 Stuff

use_proj4 {
  PROJ4PATH    = Projection/Proj4.9
  INCLUDEPATH += $$PROJ4PATH/include
#  LIBS        += -L$$PROJ4PATH/lib
#  LIBS        += -lproj_32
  DEFINES     += USE_PROJ_4
}

INCLUDEPATH	+= inc/Nodes/
INCLUDEPATH	+= Public/inc/
INCLUDEPATH	+= Public/inc/BaseNodes
INCLUDEPATH	+= inc/TinyXml
INCLUDEPATH	+= OpenCRG/baselib/inc


# want to create dependencies
DEPENDPATH = $$INCLUDEPATH

# we are building a library here
TEMPLATE = lib

HEADERS += OdrBbox.hh
HEADERS += OdrCoord.hh
HEADERS += OdrHierCoord.hh
HEADERS += OdrLaneCoord.hh
HEADERS += OdrGeoCoord.hh
HEADERS += OdrPosition.hh
HEADERS += OdrTrackCoord.hh
HEADERS += OdrPath.hh
HEADERS += OdrParserCallback.hh
HEADERS += OdrReader.hh
HEADERS += OdrReaderXML.hh
HEADERS += OdrRoadQuery.hh
HEADERS += OdrRoadData.hh
HEADERS += OdrManagerLite.hh
HEADERS += OdrManager.hh
HEADERS += OdrOpenDRIVE.hh
HEADERS += WKTParser.hh
HEADERS += WKTNode.hh

HEADERS += OdrBorder.hh
HEADERS += OdrBridge.hh
HEADERS += OdrControlEntry.hh
HEADERS += OdrController.hh
HEADERS += OdrCornerInertial.hh
HEADERS += OdrCornerReferencee.hh
HEADERS += OdrCornerRelative.hh
HEADERS += OdrCornerLocal.hh
HEADERS += OdrCornerRoad.hh
HEADERS += OdrCrossfall.hh
HEADERS += OdrDataQuality.hh
HEADERS += OdrElevation.hh
HEADERS += OdrErrorDesc.hh
HEADERS += OdrGenericNode.hh
HEADERS += OdrGeoHeader.hh
HEADERS += OdrGeoNode.hh
HEADERS += OdrHeader.hh
HEADERS += OdrJuncController.hh
HEADERS += OdrJuncHeader.hh
HEADERS += OdrJuncLaneLink.hh
HEADERS += OdrJuncLink.hh
HEADERS += OdrJuncPriority.hh
HEADERS += OdrLane.hh
HEADERS += OdrLaneAccess.hh
HEADERS += OdrLaneHeight.hh
HEADERS += OdrLaneRule.hh
HEADERS += OdrLaneLink.hh
HEADERS += OdrLaneMaterial.hh
HEADERS += OdrLaneSection.hh
HEADERS += OdrLaneSpeed.hh
HEADERS += OdrLaneOffset.hh
HEADERS += OdrLaneValidity.hh
HEADERS += OdrLaneVisibility.hh
HEADERS += OdrLaneWidth.hh
HEADERS += OdrLaneBorder.hh
HEADERS += OdrMarking.hh
HEADERS += OdrMarkings.hh
HEADERS += OdrNode.hh
HEADERS += OdrObject.hh
HEADERS += OdrObjectOutline.hh
HEADERS += OdrObjectMaterial.hh
HEADERS += OdrObjectRef.hh
HEADERS += OdrOffset.hh
HEADERS += OdrPositionInertial.hh
HEADERS += OdrPositionRoad.hh
HEADERS += OdrRawDataDesc.hh
HEADERS += OdrReference.hh
HEADERS += OdrRepeat.hh
HEADERS += OdrRoadHeader.hh
HEADERS += OdrRoadLink.hh
HEADERS += OdrRoadMark.hh
HEADERS += OdrRoadMarkExplicitLine.hh
HEADERS += OdrRoadMarkType.hh
HEADERS += OdrRoadMarkLine.hh
HEADERS += OdrRoadMarkSway.hh
HEADERS += OdrRoadType.hh
HEADERS += OdrRoadSpeed.hh
HEADERS += OdrSignal.hh
HEADERS += OdrSignalRef.hh
HEADERS += OdrSignalDep.hh
HEADERS += OdrSuperelevation.hh
HEADERS += OdrSurfaceCRG.hh
HEADERS += OdrTrafficObject.hh
HEADERS += OdrTunnel.hh
HEADERS += OdrUserData.hh
HEADERS += OdrRailroadSwitch.hh
HEADERS += OdrGeoReference.hh
HEADERS += OdrLateralShape.hh
HEADERS += OdrJuncGroup.hh
HEADERS += OdrJuncRef.hh
HEADERS += OdrParkingSpace.hh
HEADERS += OdrParkingSpaceMarking.hh
           
HEADERS += OdrGeoArc.hh
HEADERS += OdrGeoLine.hh
HEADERS += OdrGeoSpiralBase.hh
HEADERS += OdrGeoSpiral.hh
HEADERS += OdrGeoSpiralOdr.hh
HEADERS += OdrGeoPoly.hh
HEADERS += OdrGeoParamPoly.hh

SOURCES  = src/OdrBbox.cc
SOURCES += src/OdrCoord.cc
SOURCES += src/OdrHierCoord.cc
SOURCES += src/OdrLaneCoord.cc
SOURCES += src/OdrGeoCoord.cc
SOURCES += src/OdrPosition.cc
SOURCES += src/OdrTrackCoord.cc
SOURCES += src/OdrPath.cc
SOURCES += src/OdrParserCallback.cc
SOURCES += src/OdrReader.cc
SOURCES += src/OdrReaderXML.cc
SOURCES += src/OdrRoadQuery.cc
SOURCES += src/OdrRoadData.cc
SOURCES += src/OdrManagerLite.cc
SOURCES += src/OdrManager.cc
SOURCES += src/WKTParser.cc
SOURCES += src/WKTNode.cc
SOURCES += src/BaseNodes/OdrBorder.cc
SOURCES += src/BaseNodes/OdrBridge.cc
SOURCES += src/BaseNodes/OdrControlEntry.cc
SOURCES += src/BaseNodes/OdrController.cc
SOURCES += src/BaseNodes/OdrCornerInertial.cc
SOURCES += src/BaseNodes/OdrCornerReference.cc
SOURCES += src/BaseNodes/OdrCornerRelative.cc
SOURCES += src/BaseNodes/OdrCornerLocal.cc
SOURCES += src/BaseNodes/OdrCornerRoad.cc
SOURCES += src/BaseNodes/OdrCrossfall.cc
SOURCES += src/BaseNodes/OdrDataQuality.cc
SOURCES += src/BaseNodes/OdrElevation.cc
SOURCES += src/BaseNodes/OdrErrorDesc.cc
SOURCES += src/BaseNodes/OdrGenericNode.cc
SOURCES += src/BaseNodes/OdrGeoHeader.cc
SOURCES += src/BaseNodes/OdrGeoNode.cc
SOURCES += src/BaseNodes/OdrHeader.cc
SOURCES += src/BaseNodes/OdrJuncController.cc
SOURCES += src/BaseNodes/OdrJuncHeader.cc
SOURCES += src/BaseNodes/OdrJuncLaneLink.cc
SOURCES += src/BaseNodes/OdrJuncLink.cc
SOURCES += src/BaseNodes/OdrJuncPriority.cc
SOURCES += src/BaseNodes/OdrLane.cc
SOURCES += src/BaseNodes/OdrLaneAccess.cc
SOURCES += src/BaseNodes/OdrLaneHeight.cc
SOURCES += src/BaseNodes/OdrLaneRule.cc
SOURCES += src/BaseNodes/OdrLaneLink.cc
SOURCES += src/BaseNodes/OdrLaneMaterial.cc
SOURCES += src/BaseNodes/OdrLaneSection.cc
SOURCES += src/BaseNodes/OdrLaneSpeed.cc
SOURCES += src/BaseNodes/OdrLaneOffset.cc
SOURCES += src/BaseNodes/OdrLaneValidity.cc
SOURCES += src/BaseNodes/OdrLaneVisibility.cc
SOURCES += src/BaseNodes/OdrLaneWidth.cc
SOURCES += src/BaseNodes/OdrLaneBorder.cc
SOURCES += src/BaseNodes/OdrMarking.cc
SOURCES += src/BaseNodes/OdrMarkings.cc
SOURCES += src/BaseNodes/OdrNode.cc
SOURCES += src/BaseNodes/OdrObject.cc
SOURCES += src/BaseNodes/OdrObjectOutline.cc
SOURCES += src/BaseNodes/OdrObjectMaterial.cc
SOURCES += src/BaseNodes/OdrObjectRef.cc
SOURCES += src/BaseNodes/OdrOffset.cc
SOURCES += src/BaseNodes/OdrPositionInertial.cc
SOURCES += src/BaseNodes/OdrPositionRoad.cc
SOURCES += src/BaseNodes/OdrRawDataDesc.cc
SOURCES += src/BaseNodes/OdrRepeat.cc
SOURCES += src/BaseNodes/OdrReference.cc
SOURCES += src/BaseNodes/OdrRoadHeader.cc
SOURCES += src/BaseNodes/OdrRoadLink.cc
SOURCES += src/BaseNodes/OdrRoadMark.cc
SOURCES += src/BaseNodes/OdrRoadMarkExplicitLine.cc
SOURCES += src/BaseNodes/OdrRoadMarkType.cc
SOURCES += src/BaseNodes/OdrRoadMarkLine.cc
SOURCES += src/BaseNodes/OdrRoadMarkSway.cc
SOURCES += src/BaseNodes/OdrRoadType.cc
SOURCES += src/BaseNodes/OdrRoadSpeed.cc
SOURCES += src/BaseNodes/OdrSignal.cc
SOURCES += src/BaseNodes/OdrSignalRef.cc
SOURCES += src/BaseNodes/OdrSignalDep.cc
SOURCES += src/BaseNodes/OdrSuperelevation.cc
SOURCES += src/BaseNodes/OdrSurfaceCRG.cc
SOURCES += src/BaseNodes/OdrTrafficObject.cc
SOURCES += src/BaseNodes/OdrTunnel.cc
SOURCES += src/BaseNodes/OdrUserData.cc
SOURCES += src/BaseNodes/OdrRailroadSwitch.cc
SOURCES += src/BaseNodes/OdrGeoReference.cc
SOURCES += src/BaseNodes/OdrLateralShape.cc
SOURCES += src/BaseNodes/OdrJuncGroup.cc
SOURCES += src/BaseNodes/OdrJuncRef.cc
SOURCES += src/BaseNodes/OdrParkingSpace.cc
SOURCES += src/BaseNodes/OdrParkingSpaceMarking.cc
                 
SOURCES += src/Nodes/OdrGeoArc.cc
SOURCES += src/Nodes/OdrGeoLine.cc
SOURCES += src/Nodes/OdrGeoSpiralBase.cc
SOURCES += src/Nodes/OdrGeoSpiral.cc
SOURCES += src/Nodes/OdrGeoSpiralOdr.cc
SOURCES += src/Nodes/OdrGeoPoly.cc
SOURCES += src/Nodes/OdrGeoParamPoly.cc

SOURCES += src/TinyXml/tinyxml.cpp
SOURCES += src/TinyXml/tinyxmlerror.cpp
SOURCES += src/TinyXml/tinyxmlparser.cpp
SOURCES += src/TinyXml/tinystr.cpp

SOURCES += OpenCRG/baselib/src/crgContactPoint.c
SOURCES += OpenCRG/baselib/src/crgEvalpk.c
SOURCES += OpenCRG/baselib/src/crgEvaluv2xy.c
SOURCES += OpenCRG/baselib/src/crgEvalxy2uv.c
SOURCES += OpenCRG/baselib/src/crgEvalz.c
SOURCES += OpenCRG/baselib/src/crgLoader.c
SOURCES += OpenCRG/baselib/src/crgMgr.c
SOURCES += OpenCRG/baselib/src/crgMsg.c
SOURCES += OpenCRG/baselib/src/crgOptionMgmt.c
SOURCES += OpenCRG/baselib/src/crgPortability.c
SOURCES += OpenCRG/baselib/src/crgStatistics.c

#projection stuff
HEADERS += OdrProjection.hh
SOURCES += src/OdrProjection.cc

DESTDIR     = lib
TARGET      = OpenDrive

