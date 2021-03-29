cmake_minimum_required(VERSION 2.8.3)
project(exercise3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
)

find_package(OpenCV)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(map_goals_hardcoded src/map_goals_hardcoded.cpp)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(map_goals_original src/map_goals_original.cpp)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(map_goals_auto src/map_goals_auto.cpp)

target_link_libraries(map_goals_original ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(map_goals_hardcoded ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(map_goals_auto ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})