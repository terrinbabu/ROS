cmake_minimum_required(VERSION 2.8.3)
project(user_pack)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  tf
  tf_conversions
  actionlib
  message_generation
  std_msgs
  trajectory_msgs
  geometry_msgs
  genmsg
  actionlib_msgs
  moveit_msgs
  cmake_modules
  pluginlib
  tsupport
  eigen_matrix_utils
  eigen_state_space_systems
  rosdyn_ekf
  rosdyn_core
  sensor_msgs
  subscription_notifier
  )

generate_messages(DEPENDENCIES
    geometry_msgs
    actionlib_msgs
    std_msgs
    moveit_msgs
    sensor_msgs
    )

catkin_package(CATKIN_DEPENDS
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    tsupport
    eigen_matrix_utils
    eigen_state_space_systems
    rosdyn_ekf
    rosdyn_core
    sensor_msgs
    INCLUDE_DIRS include
    )

include_directories(${catkin_INCLUDE_DIRS} include ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

link_directories(${catkin_LIBRARY_DIRS})
add_executable(trial src/trial.cpp)
target_link_libraries(trial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(trial user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(moveit_check src/moveit_check.cpp)
target_link_libraries(moveit_check ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(moveit_check user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(sinusoidal_action src/sinusoidal_action.cpp)
target_link_libraries(sinusoidal_action ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(sinusoidal_action user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(planning_scene src/planning_scene.cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(planning_scene user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(low_pass_filter_example src/low_pass_filter_example.cpp)
target_link_libraries(low_pass_filter_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(low_pass_filter_example user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(wrench_filtering src/wrench_filtering.cpp)
target_link_libraries(wrench_filtering ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(wrench_filtering user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(joint_state_filtering src/joint_state_filtering.cpp)
target_link_libraries(joint_state_filtering ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(joint_state_filtering user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(write_rostopic src/write_rostopic.cpp)
target_link_libraries(write_rostopic ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(write_rostopic user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(kalman_filter_example src/kalman_filter_example.cpp)
target_link_libraries(kalman_filter_example ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(kalman_filter_example user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(joint_state_sub src/joint_state_sub.cpp)
target_link_libraries(joint_state_sub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(joint_state_sub user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(ident_trj_filtering src/ident_trj_filtering.cpp)
target_link_libraries(ident_trj_filtering ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ident_trj_filtering user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(cartesian_joint_movement src/cartesian_joint_movement.cpp)
target_link_libraries(cartesian_joint_movement ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(cartesian_joint_movement user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(virtual_force_sensor_estimation src/virtual_force_sensor_estimation.cpp)
target_link_libraries(virtual_force_sensor_estimation ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(virtual_force_sensor_estimation user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(fake_joint_state_pub src/fake_joint_state_pub.cpp)
target_link_libraries(fake_joint_state_pub ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(fake_joint_state_pub user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(kalman_filter_joint_state src/kalman_filter_joint_state.cpp)
target_link_libraries(kalman_filter_joint_state ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(kalman_filter_joint_state user_pack_gencpp)

link_directories(${catkin_LIBRARY_DIRS})
add_executable(zero_phase_filter src/zero_phase_filter.cpp)
target_link_libraries(zero_phase_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(zero_phase_filter user_pack_gencpp)
