cmake_minimum_required(VERSION 3.0)

enable_testing()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


add_subdirectory(../../../../public/src ${CMAKE_BINARY_DIR}/public)

target_include_directories(fssimplewindow PUBLIC ${HEADER_PATH})
target_include_directories(ysclass PUBLIC ${HEADER_PATH})
target_include_directories(ysbitmapfont PUBLIC ${HEADER_PATH})


add_executable(pcdSaver MACOSX_BUNDLE main.cpp)

target_link_libraries(pcdSaver fssimplewindow)
target_link_libraries(pcdSaver ysclass)
target_link_libraries(pcdSaver ysbitmapfont)