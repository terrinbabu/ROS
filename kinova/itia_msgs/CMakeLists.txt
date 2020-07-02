cmake_minimum_required(VERSION 2.8.3)
project(itia_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS
              geometry_msgs
              roscpp
              std_msgs
              kdl_conversions
              eigen_conversions
              tf_conversions
              genmsg
              message_generation
              cmake_modules
              actionlib_msgs
              actionlib
            )

find_package(orocos_kdl REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Float64Stamped.msg
  MotionStamped.msg
  MotionRequest.msg
  JointInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  send_bool.srv
  send_float64.srv
  send_string.srv
  get_config.srv
  set_config.srv
)

add_action_files(
  FILES
  interaction_task.action
  string.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs 
	geometry_msgs 
	actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp 
                 std_msgs 
                 geometry_msgs 
                 kdl_conversions 
                 eigen_conversions 
                 tf_conversions 
                 genmsg
                 cmake_modules  
                 actionlib
                 actionlib_msgs 
  DEPENDS system_lib orocos_kdl
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)
