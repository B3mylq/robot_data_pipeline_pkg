cmake_minimum_required(VERSION 3.10.2)
project(robot_data_pipeline_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
  geometry_msgs
  tf
  laser_geometry
  rosbag
)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(cppzmq REQUIRED)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RobotState.msg
  RobotCmd.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   # Service1.srv
#   # Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_data_pub_pkg
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)




# 递归引入头文件的函数
function(include_sub_directories_recursively root_dir)
	if(IS_DIRECTORY ${root_dir})
		message("include dir: " ${root_dir})
		include_directories(${root_dir})
	endif()
	file(GLOB ALL_SUB RELATIVE ${root_dir} ${root_dir}/*)
	foreach(sub ${ALL_SUB})
		if(IS_DIRECTORY ${root_dir}/${sub})
			include_sub_directories_recursively(${root_dir}/${sub})
		endif()
	endforeach()
endfunction()

include_sub_directories_recursively(${PROJECT_SOURCE_DIR}/include)
include_sub_directories_recursively(${PROJECT_SOURCE_DIR}/NetworkLib)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(ROBOT_LIB_DIR ${PROJECT_SOURCE_DIR}/include)
set(NETWORK_LIB_DIR ${PROJECT_SOURCE_DIR}/NetworkLib)

file(GLOB_RECURSE SOURCE_FILES  ${ROBOT_LIB_DIR}/*.cpp ${NETWORK_LIB_DIR}/*.cpp)
add_executable(robot_state_pub src/robot_state_pub_node.cpp ${SOURCE_FILES})
target_link_libraries(robot_state_pub 
    spdlog::spdlog_header_only
    ${catkin_LIBRARIES}
    cppzmq
)
add_dependencies(robot_state_pub ${PROJECT_NAME}_generate_messages_cpp)


file(GLOB_RECURSE SOURCE_FILES  ${ROBOT_LIB_DIR}/*.cpp ${NETWORK_LIB_DIR}/*.cpp)
add_executable(robot_state_sub src/robot_state_sub_node.cpp ${SOURCE_FILES})
target_link_libraries(robot_state_sub 
spdlog::spdlog_header_only
${catkin_LIBRARIES}
cppzmq
)
add_dependencies(robot_state_sub ${PROJECT_NAME}_generate_messages_cpp)

# file(GLOB_RECURSE SOURCE_FILES  ${ROBOT_LIB_DIR}/*.cpp ${NETWORK_LIB_DIR}/*.cpp)
# add_executable(pub_test src/pub_node_test.cpp ${SOURCE_FILES})
# target_link_libraries(pub_test 
# spdlog::spdlog_header_only
# ${catkin_LIBRARIES}
# cppzmq
# )
# add_dependencies(pub_test ${PROJECT_NAME}_generate_messages_cpp)

# file(GLOB_RECURSE SOURCE_FILES  ${ROBOT_LIB_DIR}/*.cpp ${NETWORK_LIB_DIR}/*.cpp)
# add_executable(sub_test src/sub_node_test.cpp ${SOURCE_FILES})
# target_link_libraries(sub_test 
# spdlog::spdlog_header_only
# ${catkin_LIBRARIES}
# cppzmq
# )
# add_dependencies(sub_test ${PROJECT_NAME}_generate_messages_cpp)

