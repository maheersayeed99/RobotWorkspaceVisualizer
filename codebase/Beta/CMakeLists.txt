cmake_minimum_required(VERSION 3.15.0)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Robot_Workspace)

set(INCLUDE_DIR include/)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(
  include
  ${INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_subdirectory(../../../public/src ${CMAKE_BINARY_DIR}/public)
add_subdirectory(../../../MMLPlayer/ym2612 ${CMAKE_BINARY_DIR}/ym2612)
add_subdirectory(../../../MMLPlayer/mmlplayer ${CMAKE_BINARY_DIR}/mmlplayer)


add_library(robot include/Robot.h include/Robot.cpp include/getWS.cpp)
target_link_libraries(robot Eigen3::Eigen)

add_executable(find_ws MACOSX_BUNDLE src/main.cpp)
target_link_libraries(find_ws robot)
target_link_libraries(find_ws fssimplewindow)
target_link_libraries(find_ws ysclass)
target_link_libraries(find_ws ysbitmapfont)

enable_testing()
add_subdirectory(tests)