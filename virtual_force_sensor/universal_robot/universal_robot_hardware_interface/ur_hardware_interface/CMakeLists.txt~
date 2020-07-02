cmake_minimum_required(VERSION 2.8.3)
project(ur_hardware_interface)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast -fPIC -Dlinux)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  itia_basic_hardware_interface  
  roscpp   
  sensor_msgs  
  nodelet
  tsupport
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS itia_basic_hardware_interface  roscpp sensor_msgs nodelet tsupport
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/ur_hi_nodelet.cpp
  src/${PROJECT_NAME}/ur_hardware_interface.cpp
  src/${PROJECT_NAME}/ur_driver.cpp
  src/${PROJECT_NAME}/robot_state.cpp
  src/${PROJECT_NAME}/robot_state_RT.cpp
  src/${PROJECT_NAME}/ur_communication.cpp
  src/${PROJECT_NAME}/ur_realtime_communication.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
