cmake_minimum_required(VERSION 2.8.3)
project(rosdyn_ekf)

add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosdyn_core
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rosdyn_ekf
  CATKIN_DEPENDS roscpp rosdyn_core sensor_msgs
  DEPENDS
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/rosdyn_ekf.cpp
 )
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 add_executable(${PROJECT_NAME}_node src/rosdyn_ekf_node.cpp)
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
 )



install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

