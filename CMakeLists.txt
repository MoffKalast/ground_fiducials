cmake_minimum_required(VERSION 2.8.3)
project(ground_fiducials)

find_package(catkin REQUIRED)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

install(PROGRAMS
	scripts/ground_fiducials.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

