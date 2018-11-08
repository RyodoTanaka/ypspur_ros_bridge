cmake_minimum_required(VERSION 2.8.3)
project(ypspur_ros_bridge)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
  message_generation
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ypspur_ros_bridge
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
   /usr/local/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/lib
)
## Declare a cpp executable
add_executable(ypspur_ros_bridge_driver_node src/ypspur_ros_bridge_driver_node.cpp)
add_executable(ypspur_ros_bridge_odom_publisher_node src/ypspur_ros_bridge_odom_publisher_node.cpp)

add_dependencies(
  ypspur_ros_bridge_driver_node 
  ypspur_ros_bridge_driver_node_generate_messages_cpp
)
add_dependencies(
  ypspur_ros_bridge_odom_publisher_node 
  ypspur_ros_bridge_odom_publisher_node_generate_messages_cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(
  ypspur_ros_bridge_driver_node
  ${catkin_LIBRARIES}
  ypspur
)
target_link_libraries(
  ypspur_ros_bridge_odom_publisher_node
  ypspur
  ${catkin_LIBRARIES}
)
