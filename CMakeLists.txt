cmake_minimum_required(VERSION 2.8)

project( realtimeDisparity )

set( CMAKE_VERBOSE_MAKEFILE 1) # to see all make process




# ELAS
set (LIBELAS_SRC_DIR elas)
include_directories("${LIBELAS_SRC_DIR}")
FILE(GLOB LIBELAS_SRC_FILES "elas/*.cpp")


add_executable( realtimeDisparity main.cpp ${LIBELAS_SRC_FILES} ${LIBS} )



SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-msse3 -O3 -pg -std=c++11")



# build-in packages

find_package(Eigen3)
if(EIGEN3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost 1.47 COMPONENTS system )
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  target_link_libraries(realtimeDisparity ${Boost_LIBRARIES})
endif()

find_package(OpenGL)
if(OPENGL_FOUND)
  include_directories(${OPENGL_INCLUDE_DIR})
  target_link_libraries(realtimeDisparity ${OPENGL_LIBRARIES})
endif()

find_package(CUDA)
if(CUDA_FOUND)
  include_directories(${CUDA_INCLUDE_DIRS})
  target_link_libraries(realtimeDisparity ${CUDA_LIBRARIES})
endif()

find_package(VTK)
if(VTK_FOUND)
  include_directories(${VTK_INCLUDE_DIRS})
  target_link_libraries(realtimeDisparity ${VTK_LIBRARIES})
endif()



# Point Grey FlyCapture2 SDK
include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)
target_link_libraries(realtimeDisparity flycapture )


# OpenNI used by pcl
#link_directories(/usr/lib/OpenNI2/Drivers)
#target_link_libraries(realtimeDisparity  OpenNI2 OpenNI flann_cpp)
find_library(OPENNI2 OpenNI2)
find_library(OPENNI OpenNI)
find_library(FLANN_CPP flann_cpp)




#
# use locally-installed opencv and pcl (which are latest from github)
#
set (LOCAL_INSTALL_DIR "/home/zhao/localinstall")
include_directories("${LOCAL_INSTALL_DIR}/include/" )
include_directories("${LOCAL_INSTALL_DIR}/include/opencv" )
include_directories("${LOCAL_INSTALL_DIR}/include/pcl-1.7" )
# link all *.so files directory
FILE(GLOB LIBOPENCV_FILES "${LOCAL_INSTALL_DIR}/lib/libopencv*.so" )
FILE(GLOB LIBPCL_FILES "${LOCAL_INSTALL_DIR}/lib/libpcl*.so" )
target_link_libraries(realtimeDisparity  ${LIBOPENCV_FILES} ${LIBPCL_FILES} )





