cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping)

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS 
             roscpp
             message_generation
             image_transport
             cv_bridge
             tf
             visualization_msgs
             camera_calibration_parsers)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)

include_directories(${PROJECT_SOURCE_DIR}/src/)


SET(SOURCES ${PROJECT_SOURCE_DIR}/src/main.cpp
            ${PROJECT_SOURCE_DIR}/src/aruco_mapping.cpp
	    ${PROJECT_SOURCE_DIR}/src/arucofidmarkers.cpp
	    ${PROJECT_SOURCE_DIR}/src/cvdrawingutils.cpp
	    ${PROJECT_SOURCE_DIR}/src/cameraparameters.cpp
	    ${PROJECT_SOURCE_DIR}/src/board.cpp
	    ${PROJECT_SOURCE_DIR}/src/marker.cpp
	    ${PROJECT_SOURCE_DIR}/src/boarddetector.cpp
	    ${PROJECT_SOURCE_DIR}/src/markerdetector.cpp
            ${PROJECT_SOURCE_DIR}/src/ar_omp.cpp
            ${PROJECT_SOURCE_DIR}/src/subpixelcorner.cpp
            ${PROJECT_SOURCE_DIR}/src/highlyreliablemarkers.cpp
            )
 #${PROJECT_SOURCE_DIR}/src/chromaticmask.cpp  
SET(HEADERS ${PROJECT_SOURCE_DIR}/include/aruco_mapping.h
            ${PROJECT_SOURCE_DIR}/include/ar_omp.h
            ${PROJECT_SOURCE_DIR}/include/aruco.h
            ${PROJECT_SOURCE_DIR}/include/board.h
            ${PROJECT_SOURCE_DIR}/include/boarddetector.h
            ${PROJECT_SOURCE_DIR}/include/arucofidmarkers.h
            ${PROJECT_SOURCE_DIR}/include/cameraparameters.h
            ${PROJECT_SOURCE_DIR}/include/chromaticmask.h
            ${PROJECT_SOURCE_DIR}/include/cvdrawingutils.h
            ${PROJECT_SOURCE_DIR}/include/exports.h
            ${PROJECT_SOURCE_DIR}/include/highlyreliablemarkers.h
            ${PROJECT_SOURCE_DIR}/include/marker.h
            ${PROJECT_SOURCE_DIR}/include/markerdetector.h
            ${PROJECT_SOURCE_DIR}/include/subpixelcorner.h)

add_message_files(FILES ArucoMarker.msg)

generate_messages(DEPENDENCIES
                  std_msgs
                  geometry_msgs)
   
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} aruco_mapping_gencpp )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${ROS_LIBRARIES} ${catkin_LIBRARIES})


 
