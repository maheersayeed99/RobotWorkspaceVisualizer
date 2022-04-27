find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
<<<<<<< Updated upstream
add_library(robot Robot.h Robot.cpp getWS.cpp tinyxml2.h tinyxml2.cpp)
target_link_libraries(robot Eigen3::Eigen ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(robot PUBLIC .)
=======
add_library(robot Robot.h Robot.cpp getWS.cpp tinyxml2.h tinyxml2.cpp lattice.h)
target_link_libraries(robot Eigen3::Eigen ysclass)
target_include_directories(robot PUBLIC .)
>>>>>>> Stashed changes
