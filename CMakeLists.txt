cmake_minimum_required(VERSION 2.8.3)
project(rrbt)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++11 -march=native -funroll-all-loops -fpeel-loops -ftracer -ftree-vectorize -mavx2 -mfma -fomit-frame-pointer -fpermissive")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_executable(rrbt src/rrbt.cpp src/kdtree.c)
