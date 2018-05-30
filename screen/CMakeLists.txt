cmake_minimum_required(VERSION 2.8.3)
project(tft)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Tft.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
