#version
cmake_minimum_required(VERSION 2.8.3)

#package nae
project(umd_arcl_teleop)

#find required packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs joy umd_arcl_driver)

#declare package
catkin_package()

#specify header file locations
include_directories(include ${catkin_INCLUDE_DIRS})

#add executables
add_executable(arclTeleop src/main.cpp)

#linker
target_link_libraries(arclTeleop ${catkin_LIBRARIES})
