cmake_minimum_required(VERSION 3.5)

project(visualOdometry)

#set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O2 -march=native" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

ADD_SUBDIRECTORY( ${PROJECT_SOURCE_DIR}/src )
