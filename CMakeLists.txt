cmake_minimum_required( VERSION 2.8 )
project ( Kalman_Filter )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
############### source and test ######################
set(SOURCE_FILES Kalman_Filter.cpp test.cpp)
add_executable(Kalman_Filter ${SOURCE_FILES})

