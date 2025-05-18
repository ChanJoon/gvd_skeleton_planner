cmake_minimum_required(VERSION 2.8.3)
project(gvd_skeleton)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  pcl_conversions
  voxblox
  voxblox_ros
  voxblox_skeleton
  voxblox_planning_common
  mav_msgs
  mav_path_smoothing
  mav_planning_common
  mav_trajectory_generation_ros
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/skeleton_global_planner.cpp
  src/skeleton_graph_planner.cpp
)

add_executable(skeleton_global_planner_node
  src/skeleton_global_planner_node.cpp
)

target_link_libraries(skeleton_global_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable (map_pub src/map_publisher.cpp )
target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} skeleton_global_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)