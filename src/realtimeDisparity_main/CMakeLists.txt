cmake_minimum_required(VERSION 2.8)

project( realtimeDisparity_main )

set( CMAKE_VERBOSE_MAKEFILE 1) # to see all make process


add_library ( realtimeDisparity_main OBJECT ${PROJECT_SOURCE_DIR}/src/main.cpp )







# ELAS
include_directories( ${CMAKE_SOURCE_DIR}/elas/include )


# build-in packages

find_package(Eigen3)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.47 COMPONENTS system )
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()

find_package(OpenGL)
if(OPENGL_FOUND)
  include_directories(${OPENGL_INCLUDE_DIR})
endif()

find_package(CUDA 6.0)
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
endif()

find_package(VTK 5.8)
if(VTK_FOUND)
  include_directories(${VTK_INCLUDE_DIRS})
endif()



# Point Grey FlyCapture2 SDK
include_directories("/usr/include/flycapture")








#
# use locally-installed opencv and pcl (which are latest from github)
#
set (LOCAL_INSTALL_DIR "/home/zhao/localinstall")
include_directories("${LOCAL_INSTALL_DIR}/include/" )
include_directories("${LOCAL_INSTALL_DIR}/include/opencv" )
include_directories("${LOCAL_INSTALL_DIR}/include/pcl-1.7" )


