cmake_minimum_required(VERSION 2.8.3)
project(itia_hindsight_assistance_policy)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    itia_human_prediction
)

set(CMAKE_BUILD_TYPE Debug )

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES 
    #    ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        message_runtime
    DEPENDS
        Eigen3
)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

include_directories(
    "${PROJECT_SOURCE_DIR}/src/AssistancePolicy"
    "${PROJECT_SOURCE_DIR}/src/RosSupport"
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)
add_definitions(${EIGEN_DEFINITIONS})


link_directories(
    ${catkin_LIBRARY_DIRS}
)

# file(GLOB SRC
#   "./src/AssistancePolicy/*.cpp"
# )

#add_executable(testHindsightAssistancePolicy src/robotpathgeneration.cpp.cpp ${SRC})
add_executable(robotpathgeneration "${PROJECT_SOURCE_DIR}/src/robotpathgeneration.cpp")
add_library(AssistancePolicyBase "${PROJECT_SOURCE_DIR}/src/AssistancePolicy/AssistancePolicyBase.cpp")
add_library(HuberInstance "${PROJECT_SOURCE_DIR}/src/AssistancePolicy/HuberInstance.cpp")
add_library(HuberRotationInstance "${PROJECT_SOURCE_DIR}/src/AssistancePolicy/HuberRotationInstance.cpp")
add_library(UtilFunctions "${PROJECT_SOURCE_DIR}/src/AssistancePolicy/UtilFunctions.cpp")
add_library(rosmanager "${PROJECT_SOURCE_DIR}/src/RosSupport/rosmanager.cpp")




#set(CMAKE_CXX_FLAGS "-O3")        ## Optimize

add_dependencies(robotpathgeneration ${catkin_EXPORTED_TARGETS})
target_link_libraries(robotpathgeneration AssistancePolicyBase HuberInstance HuberRotationInstance UtilFunctions rosmanager ${EIGEN_LIBRARY} ${catkin_LIBRARIES})
