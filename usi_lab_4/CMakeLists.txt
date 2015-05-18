cmake_minimum_required(VERSION 2.8.3)
project(usi_lab_4)

find_package(catkin REQUIRED COMPONENTS
	thymio_vrep
    thymio_driver
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
  CATKIN_DEPENDS thymio_vrep thymio_driver
)

###########
## Build ##
###########


#############
## Install ##
#############

