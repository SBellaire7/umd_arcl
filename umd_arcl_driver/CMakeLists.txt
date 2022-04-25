#version
cmake_minimum_required(VERSION 2.8.3)

#package name
project(umd_arcl_driver)

#find required packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs message_generation)

#add msg files
add_message_files(FILES Pose2DStamped.msg Odometer.msg Command2.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

#declare package
catkin_package(CATKIN_DEPENDS message_runtime)

#specify header file locations
include_directories(include ${catkin_INCLUDE_DIRS})

#add executables
add_executable(arclDriver src/main.cpp src/tcpClient.c src/arclUtil.c)

#linker
target_link_libraries(arclDriver ${catkin_LIBRARIES})
