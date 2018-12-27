cmake_minimum_required(VERSION 2.8.3)
project(rrbt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-all-loops")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -funroll-all-loops")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
include 
${EIGEN3_INCLUDE_DIRS} 
${BOOST_INCLUDE_DIRS} 
${OPENCV_INCLUDE_DIRS})

add_executable(rrbt src/rrbt.cpp src/kdtree.c)

target_link_libraries(rrbt
   ${OpenCV_LIBS}
)

