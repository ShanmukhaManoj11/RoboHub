cmake_minimum_required(VERSION 2.8)

project(robotics)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)

include_directories(include)

# add_executable(test_point_types tests/test_point_types.cpp)

add_executable(test_kdtree tests/test_kdtree.cpp)

# add_executable(test_compute_covariance_matrix tests/test_compute_covariance_matrix.cpp)

add_executable(test_sphere tests/test_sphere.cpp)

add_executable(test_normal_estimation tests/test_normal_estimation.cpp)

add_executable(test_plane_extraction tests/test_plane_extraction.cpp)