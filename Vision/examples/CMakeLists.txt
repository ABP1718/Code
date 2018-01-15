cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

set(CMAKE_MODULE_PATH
    ${CMAKE_SOURCE_DIR}/../cmake/modules
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

## Build paths - add example files here

## example image viewer
add_executable (DisplayImage DisplayImage.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )
## example cloud viewer
add_executable (cloud_viewer_ex cloud_viewer_example.cpp)
target_link_libraries (cloud_viewer_ex ${PCL_LIBRARIES})
## example pclvisualizer

add_executable (pclvis pclvisualizer.cpp)
target_link_libraries (pclvis ${PCL_LIBRARIES})
## example live viewer 
add_executable (live_viewer live_viewer.cpp)
target_link_libraries (live_viewer ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})
## 3d viewer
add_executable (3d_viewer 3d_viewer.cpp)
target_link_libraries (3d_viewer ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})
## example segmentation 
add_executable (segmentation segmentation.cpp)
target_link_libraries (segmentation  ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})
## example segmentation 
add_executable (3d_test 3d_test.cpp)
target_link_libraries (3d_test  ${O3D3XX_CAMERA_LIBRARIES}
                      ${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      ${O3D3XX_IMAGE_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBRARIES})
## cec
add_executable (conditional_euclidean_clustering conditional_euclidean_clustering.cpp)
target_link_libraries (conditional_euclidean_clustering ${PCL_LIBRARIES})

#rgs
add_executable (region_growing_segmentation region_growing_segmentation.cpp)
target_link_libraries (region_growing_segmentation  ${O3D3XX_CAMERA_LIBRARIES}
                      	${O3D3XX_FRAMEGRABBER_LIBRARIES}
                      	${O3D3XX_IMAGE_LIBRARIES}
			${PCL_LIBRARIES})



