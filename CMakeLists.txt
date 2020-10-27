
cmake_minimum_required(VERSION 2.8.3)

project(ferrobotics_acf)

find_package(catkin REQUIRED COMPONENTS
	rospy
	message_generation
	std_msgs)

add_message_files(
	FILES
	ACFTelem.msg
)

add_service_files(
	FILES
	SetFloat.srv
	SetDuration.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
	src/acf.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
