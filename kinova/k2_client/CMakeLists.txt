cmake_minimum_required(VERSION 2.8.3)
project(k2_client)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

find_package(Jsoncpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    std_msgs
    geometry_msgs
    cv_bridge
    image_transport
    camera_info_manager
    message_generation
)

add_message_files(FILES
    Activities.msg
    Appearance.msg
    Audio.msg    
    BodyArray.msg
    Expressions.msg
    JointOrientationAndType.msg
    JointPositionAndState.msg
    Lean.msg
    Body.msg
)

generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES
    #    ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        std_msgs
        message_runtime
    DEPENDS
        Jsoncpp
)

# Set the default path for built executables to the "bin" directory.
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
# Set the default path for built libraries to the "lib" directory.
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

include_directories(SYSTEM
    "${PROJECT_SOURCE_DIR}/include"
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIR}
)

link_directories(
    ${catkin_LIBRARY_DIRS}
)

add_executable(startAudio "${PROJECT_SOURCE_DIR}/src/startAudio.cpp")
add_executable(startBody  "${PROJECT_SOURCE_DIR}/src/startBody.cpp")
add_executable(startDepth "${PROJECT_SOURCE_DIR}/src/startDepth.cpp")
add_executable(startIR    "${PROJECT_SOURCE_DIR}/src/startIR.cpp")
add_executable(startRGB   "${PROJECT_SOURCE_DIR}/src/startRGB.cpp")

add_dependencies(startAudio ${k2_client_EXPORTED_TARGETS})
add_dependencies(startBody  ${k2_client_EXPORTED_TARGETS})
add_dependencies(startDepth ${k2_client_EXPORTED_TARGETS})
add_dependencies(startIR    ${k2_client_EXPORTED_TARGETS})
add_dependencies(startRGB   ${k2_client_EXPORTED_TARGETS})

target_link_libraries(startAudio ${JSONCPP_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(startBody  ${JSONCPP_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(startDepth ${JSONCPP_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(startIR    ${JSONCPP_LIBRARY} ${catkin_LIBRARIES})
target_link_libraries(startRGB   ${JSONCPP_LIBRARY} ${catkin_LIBRARIES})



