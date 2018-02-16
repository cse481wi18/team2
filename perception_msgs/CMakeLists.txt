cmake_minimum_required(VERSION 2.8.3)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  FILES
  ObjectFeatures.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)