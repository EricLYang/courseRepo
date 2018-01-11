cmake_minimum_required( VERSION 2.8 )
project( eigenDemo )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

# set output path
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# include eigen
include_directories(${PROJECT_SOURCE_DIR}/eigenLibrary)


add_executable( eigenDemo src/mainEigen.cpp )
