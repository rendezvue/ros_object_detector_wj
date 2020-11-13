project(rdv_orange_detection_with_realsense)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

set(APPENDED_SOURCE)

# 하위 디렉토리 추가
add_subdirectory(src)
include_directories( ./src )


# 추가 할 파일의 확장자
file(GLOB SOURCES_IN_CURRENT_DIR   "${CMAKE_CURRENT_LIST_DIR}/*.cpp")
file(GLOB HEADER_IN_CURRENT_DIR   "${CMAKE_CURRENT_LIST_DIR}/*.h" 
                                  "${CMAKE_CURRENT_LIST_DIR}/*.hpp")
                                    
set(APPENDED_SOURCE ${SOURCES_IN_CURRENT_DIR} ${APPENDED_SOURCE})
set(APPENDED_HEADER ${HEADER_IN_CURRENT_DIR} ${APPENDED_HEADER})

foreach(SOURCE ${APPENDED_SOURCE})
    message("${SOURCE}")
endforeach(SOURCE)

foreach(SOURCE ${APPENDED_SOURCE})
    file(RELATIVE_PATH RELATIVE ${PROJECT_SOURCE_DIR} ${SOURCE})
    
    get_filename_component(FILENAME ${RELATIVE} NAME)
    get_filename_component(PARENT_DIRECTORY ${RELATIVE} DIRECTORY)

    message(STATUS "${FILENAME} in ${PARENT_DIRECTORY}")    
    source_group(${PARENT_DIRECTORY} FILES ${SOURCE})
endforeach(SOURCE)

add_executable( rdv_orange_detection_with_realsense ${APPENDED_SOURCE} )

target_link_libraries(rdv_orange_detection_with_realsense ${OpenCV_LIBS})
target_link_libraries(rdv_orange_detection_with_realsense ${Boost_LIBRARIES})
target_link_libraries(rdv_orange_detection_with_realsense ${REALSENSE2_FOUND})
target_link_libraries(rdv_orange_detection_with_realsense ${DARKNET_LIB_NAME})
