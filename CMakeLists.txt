cmake_minimum_required(VERSION 2.8)


project( realtimeDisparity )

SET(CMAKE_CXX_FLAGS "-msse3 -O3")

# directories
set (LIBELAS_SRC_DIR elas)

# include directory
include_directories("${LIBELAS_SRC_DIR}")
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories("/usr/include/flycapture")

find_library(FLYCAPTURE2 flycapture)
find_package( OpenCV REQUIRED )

FILE(GLOB LIBELAS_SRC_FILES "elas/*.cpp")

add_executable( realtimeDisparity main.cpp ${LIBELAS_SRC_FILES})
##add_executable( OutputImage FlyCapture2Test.cpp )
target_link_libraries( realtimeDisparity ${OpenCV_LIBS} flycapture )
##target_link_libraries( OutputImage ${OpenCV_LIBS} flycapture )

