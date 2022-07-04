cmake_minimum_required(VERSION 3.0)
project(vtd)

# set(CMAKE_CXX_STANDARD 11)
add_definitions("-std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/../Public/inc)
include_directories(${PROJECT_SOURCE_DIR}/../Public/inc/BaseNodes)
include_directories(${PROJECT_SOURCE_DIR}/../inc)
include_directories(${PROJECT_SOURCE_DIR}/../inc/Nodes)
include_directories(${PROJECT_SOURCE_DIR}/../inc/TinyXml)
include_directories(${PROJECT_SOURCE_DIR}/../OpenCRG/baselib/inc)

file( GLOB sourcesLite 
      ../src/*.cc
      ../src/BaseNodes/*.cc
      ../src/Nodes/*.cc
      ../src/TinyXml/*.cpp
      ../OpenCRG/baselib/src/*.c )

# set(ODRMANGER_LIB ${PROJECT_SOURCE_DIR}/../build/libOpenDrive.dll)
add_executable(hello hello.cpp ${sourcesLite})
# target_link_libraries(hello ${ODRMANGER_LIB})