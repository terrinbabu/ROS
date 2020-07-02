cmake_minimum_required(VERSION 2.8.3)
project(tsupport)

add_definitions( "-std=c++11" )

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  tf
  tf_conversions
  message_generation
  actionlib
  trajectory_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  moveit_msgs
  cmake_modules
  pluginlib
  subscription_notifier)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    actionlib_msgs
    std_msgs
    moveit_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tsupport
  CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    subscription_notifier)

include_directories( 
    ${catkin_INCLUDE_DIRS} include
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    
add_library(tsupport src/tsupport.cpp )
add_dependencies( tsupport ${catkin_EXPORTED_TARGETS} )

link_directories(${catkin_LIBRARY_DIRS})
add_executable(topic_average src/topic_average.cpp)
target_link_libraries(topic_average ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(topic_average tsupport_gencpp)

