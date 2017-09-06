cmake_minimum_required(VERSION 2.8.3)
project(opencvros)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  std_msgs
  image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})


## Declare the opencvros_node executable so it can be executed with "rosrun opencvros opencvros_node"
add_executable(opencvros_node src/opencvros.cpp)
target_link_libraries(opencvros_node ${catkin_LIBRARIES})
add_dependencies(opencvros_node opencv_generate_messages_cpp)

## Declare the driving_node executable so it can be executed with "rosrun opencvros driving_node"
add_executable(driving_node src/driving.cpp)
target_link_libraries(driving_node ${catkin_LIBRARIES})
add_dependencies(driving_node opencv_generate_messages_cpp)

