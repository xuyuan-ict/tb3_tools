################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(tb3_safe_teleop)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
	rospy std_msgs sensor_msgs geometry_msgs message_generation)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# catkin specific configuration
################################################################################
catkin_package(
	CATKIN_DEPENDS 
	rospy std_msgs sensor_msgs geometry_msgs)

################################################################################
# catkin specific configuration
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)


################################################################################
# Install
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
install(PROGRAMS
  scripts/tb3-safe-teleop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
