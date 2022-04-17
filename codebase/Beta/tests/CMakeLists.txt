# Forward Kinamatics
add_executable(test_fk1 MACOSX_BUNDLE test_fk1.cpp)
target_link_libraries(test_fk1 robot)
add_test(NAME test_fk1 COMMAND test_fk1)

add_executable(test_fk2 MACOSX_BUNDLE test_fk2.cpp)
target_link_libraries(test_fk2 robot)
add_test(NAME test_fk2 COMMAND test_fk2)

add_executable(test_fk3 MACOSX_BUNDLE test_fk3.cpp)
target_link_libraries(test_fk3 robot)
add_test(NAME test_fk3 COMMAND test_fk3)

# URDF
add_executable(test_urdf1 MACOSX_BUNDLE test_urdf1.cpp)
target_link_libraries(test_urdf1 robot)
add_test(NAME test_urdf1 COMMAND test_urdf1)

add_executable(test_urdf2 MACOSX_BUNDLE test_urdf2.cpp)
target_link_libraries(test_urdf2 robot)
add_test(NAME test_urdf2 COMMAND test_urdf2)

add_executable(test_urdf3 MACOSX_BUNDLE test_urdf3.cpp)
target_link_libraries(test_urdf3 robot)
add_test(NAME test_urdf3 COMMAND test_urdf3)

# Get workspace
add_executable(test_ws1 MACOSX_BUNDLE test_ws1.cpp)
target_link_libraries(test_ws1 robot)
add_test(NAME test_ws1 COMMAND test_ws1)

add_executable(test_ws2 MACOSX_BUNDLE test_ws2.cpp)
target_link_libraries(test_ws2 robot)
add_test(NAME test_ws2 COMMAND test_ws2)

add_executable(test_ws3 MACOSX_BUNDLE test_ws3.cpp)
target_link_libraries(test_ws3 robot)
add_test(NAME test_ws3 COMMAND test_ws3)