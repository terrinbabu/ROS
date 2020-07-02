cmake_minimum_required(VERSION 2.8.3)
project(itia_kinova)

add_definitions( "-std=c++11" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  genmsg
  actionlib
  actionlib_msgs
  moveit_msgs
  cmake_modules
  pluginlib
  moveit_simple_grasps
  itia_human_prediction
  kinova_msgs
  apriltags
  sound_play
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system filesystem date_time)

add_message_files(DIRECTORY msg
  FILES
    PoseArrays.msg
)

# # add_service_files(
# #   FILES
# # )

generate_messages(DEPENDENCIES
  geometry_msgs
  actionlib_msgs
  std_msgs
  moveit_msgs
  kinova_msgs
)

# Catkin
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
    
    moveit_core
    moveit_ros_planning_interface
    interactive_markers

  INCLUDE_DIRS include
)

## Build

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories( ${catkin_INCLUDE_DIRS} include)
include_directories( ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
include_directories(${itia_futils_INCLUDE_DIRS})


link_directories(${catkin_LIBRARY_DIRS})
add_executable(grasp src/grasp.cpp)
target_link_libraries(grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(grasp itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(move src/move.cpp)
target_link_libraries(move ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(move itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(trial src/trial.cpp)
target_link_libraries(trial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(trial itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(pomdp_publishers src/pomdp_publishers.cpp)
target_link_libraries(pomdp_publishers ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pomdp_publishers itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(pomdp src/pomdp.cpp)
target_link_libraries(pomdp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pomdp itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(tf_to_hand src/tf_to_hand.cpp)
target_link_libraries(tf_to_hand ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(tf_to_hand itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(voice src/voice.cpp)
target_link_libraries(voice ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(voice itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(precedence_const src/precedence_const.cpp)
target_link_libraries(precedence_const ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(precedence_const itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(object_tracking src/object_tracking.cpp)
target_link_libraries(object_tracking ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(object_tracking itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(pomdp_tsr src/pomdp_tsr.cpp)
target_link_libraries(pomdp_tsr ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pomdp_tsr itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(pomdp_stop src/pomdp_stop.cpp)
target_link_libraries(pomdp_stop ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pomdp_stop itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(pomdp_tsr_with_constraint src/pomdp_tsr_with_constraint.cpp)
target_link_libraries(pomdp_tsr_with_constraint ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(pomdp_tsr_with_constraint itia_kinova_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(calculate_length src/calculate_length.cpp)
target_link_libraries(calculate_length ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(calculate_length itia_kinova_gencpp)
