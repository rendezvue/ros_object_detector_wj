project(ros_object_detector)

cmake_minimum_required(VERSION 3.0.2)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  image_transport 
  cv_bridge
  
)
#realsense
find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()

#opencv
find_package(OpenCV REQUIRED)
include_directories( ${OPENCV_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

#boost
find_package(Boost REQUIRED COMPONENTS system thread filesystem regex date_time program_options)
#find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${Boost_LIBRARY_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ) 

# Path to Darknet project folder.
set(DARKNET_PATH ${CMAKE_CURRENT_SOURCE_DIR}/module/darknet)
# Give a custom name to shared library which is provided by Darkned when the one compiled.
set(DARKNET_LIB_NAME libdarknet)
# Link Darknet library' include folder.
include_directories(${DARKNET_PATH}/include)
# Define Darknet library' shared library.
add_library(${DARKNET_LIB_NAME} SHARED IMPORTED)
# Set a path to Darknet library' shared library
set_property(TARGET ${DARKNET_LIB_NAME} PROPERTY IMPORTED_LOCATION "${DARKNET_PATH}/libdarknet.so")

#src
set(SRC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)
FILE(GLOB SRC_SRC "${SRC_PATH}/*.cpp" ) 
include_directories(${SRC_PATH})

include_directories(${catkin_INCLUDE_DIRS})


## Generate services in the 'srv' folder
add_service_files(
   FILES
   SrvEnsemble.srv
#   Service2.srv
)

add_message_files(
   FILES
   yolo.msg
)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs   
)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES ros_ensemble_service
 CATKIN_DEPENDS roscpp std_msgs message_runtime
# CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)
#link_directories(${catkin_LIBRARY_DIRS})

#Example src
set(EXAMPLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/example)
FILE(GLOB SERVER_SRC "${EXAMPLE_PATH}/server.cpp" ) 
FILE(GLOB CLIENT_SRC "${EXAMPLE_PATH}/client.cpp" ) 
FILE(GLOB SERVER_SRC_CUTTER "${EXAMPLE_PATH}/cutter_server.cpp" ) 
FILE(GLOB CLIENT_SRC_CUTTER "${EXAMPLE_PATH}/cutter_client.cpp" ) 

include_directories(${EXAMPLE_PATH})

add_executable( ros_cutter_detector_server ${SRC_SRC} ${SERVER_SRC_CUTTER} )
add_executable( ros_cutter_detector_client ${SRC_SRC} ${CLIENT_SRC_CUTTER} )
add_executable( ros_object_detector_server ${SRC_SRC} ${SERVER_SRC} )
add_executable( ros_object_detector_client ${SRC_SRC} ${CLIENT_SRC} )


target_link_libraries(ros_object_detector_server 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${REALSENSE2_FOUND}
    ${DARKNET_LIB_NAME}
)

target_link_libraries(ros_object_detector_client     
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${REALSENSE2_FOUND}
    ${DARKNET_LIB_NAME}
)

target_link_libraries(ros_cutter_detector_server 
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${REALSENSE2_FOUND}
    ${DARKNET_LIB_NAME}
)
target_link_libraries(ros_cutter_detector_client
    ${catkin_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${REALSENSE2_FOUND}
    ${DARKNET_LIB_NAME}
)


install(TARGETS ros_cutter_detector_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(TARGETS ros_cutter_detector_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  

install(TARGETS ros_object_detector_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(TARGETS ros_object_detector_client
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})