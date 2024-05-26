cmake_minimum_required(VERSION 3.7) #same as librobotcontrol

project(fiprocket)#flying inverted pendulum

include_directories(include)

add_subdirectory(${CMAKE_SOURCE_DIR}/../librobotcontrol ${CMAKE_BINARY_DIR}/librobotcontrol_build)

# Include header files from librobotcontrol
add_subdirectory(${CMAKE_SOURCE_DIR}/../librobotcontrol/library)
add_executable(hop_test src/hop_test.cpp)


target_link_libraries(fiprocket librobotcontrol)