cmake_minimum_required(VERSION 2.8.3)
project(itia_occupancy_volume)

add_definitions("-std=c++11 -Wall")
set(CMAKE_BUILD_TYPE Release )

find_package(catkin REQUIRED COMPONENTS  roscpp tf itia_butils itia_rutils itia_tfutils std_msgs geometry_msgs message_generation)
find_package(PCL    REQUIRED )
find_package(Boost  REQUIRED COMPONENTS system program_options filesystem REQUIRED )

add_message_files(
  FILES
  GridNode.msg
  GridNodes.msg
)

add_service_files(
  FILES
  GetGridNodes.srv
  LoadOccupancyGrid.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_occupancy_volume itia_occupancy_volume_utils itia_pcl_utils
  CATKIN_DEPENDS roscpp tf itia_butils itia_rutils itia_tfutils std_msgs geometry_msgs
  DEPENDS system_lib PCL
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_library(itia_occupancy_volume_utils src/itia_occupancy_volume/utils.cpp
                                        src/itia_occupancy_volume/utils_xmlrpc.cpp
                                        src/itia_occupancy_volume/utils_yaml.cpp )
target_link_libraries(itia_occupancy_volume_utils ${catkin_LIBRARIES} ) 
add_dependencies(itia_occupancy_volume_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
                                        
add_library(itia_pcl_utils              src/itia_occupancy_volume/pcl_utils.cpp )
add_dependencies( itia_pcl_utils        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_library(itia_occupancy_volume       src/itia_occupancy_volume/input_utils.cpp 
                                        src/itia_occupancy_volume/occupancy_grid_base.cpp
                                        src/itia_occupancy_volume/occupancy_grid.cpp
                                        src/itia_occupancy_volume/occupancy_grid_set.cpp )
add_dependencies( itia_occupancy_volume ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

link_directories(${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS} )

add_executable  (itia_occupancy_volume_create_node src/itia_occupancy_volume_create_node.cpp)
add_dependencies( itia_occupancy_volume_create_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(itia_occupancy_volume_create_node itia_occupancy_volume 
                                                        itia_occupancy_volume_utils 
                                                        itia_pcl_utils 
                                                        ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} sisl yaml-cpp  )
                                                  

add_executable  (itia_occupancy_volume_query_node src/itia_occupancy_volume_query_node.cpp)
add_dependencies( itia_occupancy_volume_query_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(itia_occupancy_volume_query_node  itia_occupancy_volume 
                                                        itia_occupancy_volume_utils 
                                                        ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} sisl yaml-cpp  )



