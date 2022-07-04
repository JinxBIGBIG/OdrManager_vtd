cmake_minimum_required(VERSION 3.0)
project(roadInfos)

# set(CMAKE_CXX_STANDARD 11)
add_definitions("-std=c++11")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(OdrManager include)

set(ODRMANGER_LIB ${PROJECT_SOURCE_DIR}/lib/lib81/libODrive.1.5.5.a)

add_subdirectory(examples)
