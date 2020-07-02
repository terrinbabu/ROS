cmake_minimum_required(VERSION 2.8.3)
project(itia_gutils)
SET(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  itia_futils
  cmake_modules
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED COMPONENTS)
find_package(orocos_kdl REQUIRED)
# find_library(OROCOS_KDL orocos-kdl)
# set(orocos_kdl_LIBRARIES ${OROCOS_KDL})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_gutils
  CATKIN_DEPENDS geometry_msgs itia_futils roscpp std_msgs cmake_modules
  DEPENDS system_lib orocos_kdl Eigen3
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  include
)

add_library(itia_gutils src/itia_gutils.cpp )

add_dependencies( itia_gutils ${catkin_EXPORTED_TARGETS} 
                              ${orocos_kdl_LIBRARIES} )
                              
target_link_libraries(itia_gutils
                      ${orocos_kdl_LIBRARIES} )
                              
add_executable( itia_gutils_node
                src/itia_gutils_node.cpp )

add_dependencies( itia_gutils_node ${catkin_EXPORTED_TARGETS} )

target_link_libraries(itia_gutils_node
                      itia_gutils
                      ${orocos_kdl_LIBRARIES}
                      ${Eigen3_LIBRARIES}  
                      ${catkin_LIBRARIES}     )
