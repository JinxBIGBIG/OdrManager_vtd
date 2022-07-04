cmake_minimum_required(VERSION 2.6.0)

set(CMAKE_CONFIGURATION_TYPES Debug;Release CACHE STRING "" FORCE)

project(OdrManager_vtd)

#set( CMAKE_VERBOSE_MAKEFILE on )


# read current library version from file CURRENT_VERSION
file( STRINGS CURRENT_VERSION ODR_VERSION )


set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG")
 

# Preprocessor definitions
add_definitions( -DSWREVISION="version-${ODR_VERSION}" )
#add_definitions( -DUSE_PROJ_4 )

if( UNIX )
    add_definitions( -DLINUX )

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ansi -Wall -Wno-misleading-indentation -fsigned-char" )
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall" )
elseif( WIN32 )
  add_definitions( -DWIN32 )
	add_definitions( -D_USE_MATH_DEFINES )
	# add_definitions( /W1 ) # Only show severe warnings
	# add_definitions( /MP ) # Turn on multi processor compilation
endif()


include_directories( inc/ )
include_directories( inc/Nodes/ )
include_directories( Public/inc/ )
include_directories( Public/inc/BaseNodes )
include_directories( inc/TinyXml )
include_directories( OpenCRG/baselib/inc )
include_directories( Projection/Proj4.9/include )


# sourcesLite: Everything except OdrManager.cc and unused sources
file( GLOB sourcesLite 
      src/*.cc
      src/BaseNodes/*.cc
      src/Nodes/*.cc
      src/TinyXml/*.cpp
      OpenCRG/baselib/src/*.c )


# Remove unused sources
list( REMOVE_ITEM sourcesLite 
      ${CMAKE_CURRENT_SOURCE_DIR}/src/Player.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/src/OdrManager.cc )

# Force compilation with C++ compiler 
set_source_files_properties( ${sourcesLite} PROPERTIES LANGUAGE CXX )


# Activate rpath
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)


if( UNIX )
  set( CMAKE_SHARED_LIBRARY_SUFFIX ${CMAKE_SHARED_LIBRARY_SUFFIX}.${ODR_VERSION} )
	set( LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/ )
endif()


# Build .so/.dll
add_library( OpenDrive SHARED ${sourcesLite} src/OdrManager.cc )


set( ODRIVE_LIB ODrive.${ODR_VERSION} )

# Build .a
add_library( ${ODRIVE_LIB} STATIC ${sourcesLite} src/OdrManager.cc )


if( UNIX )
    # Set rpath to .. relative to binary
    SET_TARGET_PROPERTIES( OpenDrive PROPERTIES INSTALL_RPATH "\$ORIGIN/..")
elseif ( WIN32 )  
  set_target_properties( ${ODRIVE_LIB} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib/ )
	set_target_properties( ${ODRIVE_LIB} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib/ )

	set_target_properties( OpenDrive PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/lib/ )
	set_target_properties( OpenDrive PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/lib/ )
endif ( )
