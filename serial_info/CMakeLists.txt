cmake_minimum_required(VERSION 2.8.3)
project(serial_info)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	rospy
	serial
	std_msgs
	message_generation
)

add_message_files(
	FILES
	SerialInfo.msg
)
generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime	
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(serial_port src/serial_port.cpp)
target_link_libraries(serial_port ${catkin_LIBRARIES} )

