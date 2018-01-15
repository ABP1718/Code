cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ROBOT)

set(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/../Vision/cmake/modules
    ${CMAKE_MODULE_PATH}
    )

################################################
## Bring in dependent projects
################################################
find_package(Boost REQUIRED COMPONENTS system)
find_package(o3d3xx_camera REQUIRED)
find_package(o3d3xx_framegrabber REQUIRED)
find_package(o3d3xx_image REQUIRED)
find_package(PCL 1.7.1 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_library(LIB_boost_system NAMES boost_system)

################################################
## Manage our compiler and linker flags
################################################
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -std=c++11")

include_directories(
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/urlib
  )
link_directories(
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/urlib
  )
add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
  )

add_library (urlib 
		${CMAKE_SOURCE_DIR}/../Vision/AerobicVision.cpp
		${CMAKE_SOURCE_DIR}/urlib/UR_Control.cpp
		${CMAKE_SOURCE_DIR}/urlib/UR3_Lib.cpp
 		${CMAKE_SOURCE_DIR}/urlib/ur_communication.cpp
		${CMAKE_SOURCE_DIR}/urlib/ur_realtime_communication.cpp
		${CMAKE_SOURCE_DIR}/urlib/robot_state.cpp
		${CMAKE_SOURCE_DIR}/urlib/robot_state_RT.cpp)

target_link_libraries (urlib ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES}
		      )


## Build paths - add project files here

## 2d depth image blob detector
add_executable (main Main.cpp)
target_link_libraries (main urlib)


