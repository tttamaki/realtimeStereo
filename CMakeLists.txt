cmake_minimum_required(VERSION 2.8)

project( realtimeDisparity )

SET(CMAKE_CXX_FLAGS "-msse3 -O3 -pg -std=c++11")


include_directories("/usr/include/vtk-5.8")
include_directories("/usr/include/eigen3")
find_package(Boost 1.47 COMPONENTS system REQUIRED)
find_package(OpenGL)
find_package(CUDA)



include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)


# ELAS
set (LIBELAS_SRC_DIR elas)
include_directories("${LIBELAS_SRC_DIR}")
FILE(GLOB LIBELAS_SRC_FILES "elas/*.cpp")




set (LOCAL_INSTALL_DIR "/home/zhao/localinstall")
include_directories("${LOCAL_INSTALL_DIR}/include/" )
include_directories("${LOCAL_INSTALL_DIR}/include/opencv" )
include_directories("${LOCAL_INSTALL_DIR}/include/pcl-1.7" )
FILE(GLOB LIBOPENCV_FILES "${LOCAL_INSTALL_DIR}/lib/libopencv*.so" )
FILE(GLOB LIBPCL_FILES "${LOCAL_INSTALL_DIR}/lib/libpcl*.so" )
set (LIBS ${LIBS} ${LIBOPENCV_FILES} ${LIBPCL_FILES} )


SET(CMAKE_LD_FLAGS ${CMAKE_LD_FLAGS} "-L${LOCAL_INSTALL_DIR}/lib -L/usr/lib/OpenNI2/Drivers -lOpenNI2 -lDummyDevice -lOniFile -lPS1080  -lOpenNI -lflann_cpp")
SET(CMAKE_LD_FLAGS ${CMAKE_LD_FLAGS} "-ltbb -lrt -lpthread -lm -ldl")

add_executable( realtimeDisparity main.cpp ${LIBELAS_SRC_FILES})
target_link_libraries( realtimeDisparity ${LIBS} boost_system flycapture )

