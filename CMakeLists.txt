cmake_minimum_required(VERSION 2.8.3)
project(imu_odometer)
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	tf
	nav_msgs)

find_package(Boost REQUIRED COMPONENTS signals filesystem system)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(imu_odom_node src/imu_odom_node.cpp )
target_link_libraries(imu_odom_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})