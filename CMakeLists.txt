#############################################################################
## Set minimum required version of cmake, project name and compile options ##
#############################################################################
cmake_minimum_required(VERSION 2.8.3)
project(hal_hydrabox)

#############################################################################
## Find catkin packages and libraries for catkin and system dependencies   ##
#############################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  agribot_msgs
  agribot_common
  sensor_msgs
  tf2_geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES can
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#############################################################################
## Build                                                                   ##
#############################################################################
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${agribot_common_SOURCE_DIR}/include
)

link_directories("${PROJECT_SOURCE_DIR}")
add_executable(hydrabox_node src/Hydrabox.cpp src/hydrabox_node.cpp)
target_link_libraries(hydrabox_node ${catkin_LIBRARIES} agribot_common)

add_dependencies(hydrabox_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############


#############
## Testing ##
#############
