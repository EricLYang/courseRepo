CMAKE_MINIMUM_REQUIRED( VERSION 2.8)
PROJECT(sophus_demo)
################################################################################
# Sophus source dir
set( Sophus_SOURCE_DIR "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus")

################################################################################
# Sophus build dir
set( Sophus_DIR "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus/build")

################################################################################
set( Sophus_INCLUDE_DIR  "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus;/usr/include/eigen3" )
set( Sophus_INCLUDE_DIRS  "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus;/usr/include/eigen3" )

set( Sophus_LIBRARIES    "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus/build/libSophus.so" )
set( Sophus_LIBRARY      "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus/build/libSophus.so" )

set( Sophus_LIBRARY_DIR  "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus/build" )
set( Sophus_LIBRARY_DIRS "/home/ericyanng/ROS_WORKSPACE/slamCourse/2_class/sophus_example/Sophus/build" )
FIND_PACKAGE(sophus REQUIRED)
INCLUDE_DIRECTORIES( BEFORE ${Sophus_INCLUDE_DIR} ${PROJECT_SOURCE_DIR}/eigenLibrary)
add_executable(sophus_demo src/mainSophus.cpp)
TARGET_LINK_LIBRARIES(sophus_demo ${Sophus_LIBRARIES})

