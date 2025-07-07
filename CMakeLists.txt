cmake_minimum_required(VERSION 3.0.2)
project(gvd_skeleton_planner)

# Build type and compiler flags
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# Define voxblox_skeleton sources
set(VOXBLOX_SKELETON_SRCS
  include/voxblox_skeleton/skeleton.cpp
  include/voxblox_skeleton/skeleton_generator.cpp
  include/voxblox_skeleton/skeleton_planner.cpp
  include/voxblox_skeleton/skeleton_serialization.cpp
  include/voxblox_skeleton/sparse_graph_planner.cpp
  include/voxblox_skeleton/voxel_template_matcher.cpp
)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  pcl_conversions
  voxblox
  voxblox_ros
  voxblox_msgs
)
find_package(PCL REQUIRED)

# Export package information
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp visualization_msgs pcl_conversions voxblox voxblox_ros voxblox_msgs
  DEPENDS PCL
)

###########
## Build ##
###########

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Build the library
add_library(${PROJECT_NAME}
  src/gvd_skeleton_planner.cpp
  ${VOXBLOX_SKELETON_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Build executables
add_executable(gvd_skeleton_planner_node
  src/gvd_skeleton_planner_node.cpp
)

target_link_libraries(gvd_skeleton_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(map_pub 
  src/map_publisher.cpp
)

target_link_libraries(map_pub
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} gvd_skeleton_planner_node map_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ include/voxblox_skeleton/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch and config files
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)