cmake_minimum_required (VERSION 3.8)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add public libraries

# Define executable & target link libraries
add_executable(urdf_parser urdf_parser.cpp tinyxml2.cpp tinyxml2.h)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})