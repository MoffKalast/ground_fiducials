cmake_minimum_required(VERSION 2.8.3)
project(ground_fiducials)

find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs
)

install(PROGRAMS
	scripts/ground_fiducials.py
	scripts/smooth_fiducials.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

