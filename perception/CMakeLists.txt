cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rosbag       
  roscpp    
  rospy
  sensor_msgs
  tf
)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(perception_save_cloud_main src/save_cloud_main.cpp)
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(perception_save_cloud_main
   ${catkin_LIBRARIES}
)