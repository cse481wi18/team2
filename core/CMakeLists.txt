cmake_minimum_required(VERSION 2.8.3)
project(core)

find_package(catkin REQUIRED COMPONENTS
  fetch_api
  joint_state_reader
  perception
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS fetch_api joint_state_reader perception roscpp rospy
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)