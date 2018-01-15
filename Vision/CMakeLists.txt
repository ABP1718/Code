cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

set(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/cmake/modules
    ${CMAKE_MODULE_PATH}
    )

################################################
## Bring in dependent projects
################################################
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
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ${O3D3XX_IMAGE_INCLUDE_DIRS}
  ${O3D3XX_CAMERA_INCLUDE_DIRS}
  ${O3D3XX_FRAMEGRABBER_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )
link_directories(
  ${O3D3XX_IMAGE_LIBRARY_DIR}
  ${O3D3XX_CAMERA_LIBRARY_DIR}
  ${O3D3XX_FRAMEGRABBER_LIBRARY_DIR}
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  )
add_definitions(
  ${PCL_DEFINITIONS}
  ${OpenCV_DEFINITIONS}
  )
add_library(visionlib
	${CMAKE_SOURCE_DIR}/AerobicVision.cpp
)
target_link_libraries (visionlib ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES}
		      )

## Build paths - add example files here

## example segmentation 
add_executable (main main.cpp)
target_link_libraries (main ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES}
		      visionlib)

##add_executable (AerobicVision AerobicVision.cpp)
##target_link_libraries (AerobicVision ${O3D3XX_CAMERA_LIBRARIES}
##                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
##                      ${O3D3XX_IMAGE_LIBRARIES}
##                      ${PCL_LIBRARIES}
##                      ${OpenCV_LIBRARIES})




