cmake_minimum_required(VERSION 2.8.3)
project(thymio_vrep)

find_package(catkin REQUIRED COMPONENTS
	rospy
	asebaros
	std_msgs
	rosgraph_msgs
	#vrep_common
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy asebaros std_msgs message_runtime  rosgraph_msgs #vrep_common
)

###########
## Build ##
###########


#############
## Install ##
#############

