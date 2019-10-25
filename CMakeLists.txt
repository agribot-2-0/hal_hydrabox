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
 ${CMAKE_SOURCE_DIR}/agribot_common/include
)
link_directories("${PROJECT_SOURCE_DIR}")
add_executable(hydrabox_node ${CMAKE_SOURCE_DIR}/agribot_common/src/Can.cpp src/Hydrabox.cpp src/hydrabox_node.cpp)
target_link_libraries(hydrabox_node ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

