# cmake_minimum_required(VERSION 3.7) #same as librobotcontrol

# project(fiprocket)#flying inverted pendulum

# include_directories(${CMAKE_SOURCE_DIR}/include)

# add_subdirectory(${CMAKE_SOURCE_DIR}/../librobotcontrol ${CMAKE_BINARY_DIR}/librobotcontrol_build)

# # Include header files from librobotcontrol
# add_subdirectory(${CMAKE_SOURCE_DIR}/../librobotcontrol/library ${CMAKE_BINARY_DIR}/librobotcontrol/library_build)
# add_executable(hop_test src/hop_test.cpp)


# target_link_libraries(fiprocket librobotcontrol)






cmake_minimum_required(VERSION 3.16)

project(VerticalLanderBangBang)

add_subdirectory(${CMAKE_SOURCE_DIR}/../librobotcontrol ${CMAKE_BINARY_DIR}/librobotcontrol_build)

include_directories(include)

add_executable(accel_test src/accel_test.cpp)
target_link_libraries(accel_test robotics_cape)

add_executable(hop_test src/hop_test.cpp)
target_link_libraries(hop_test robotics_cape)

add_executable(hop_test_v0 src/hop_test_v0.cpp)
target_link_libraries(hop_test_v0 robotics_cape)

add_executable(prop_system_id src/prop_system_id.cpp)
target_link_libraries(prop_system_id robotics_cape)
