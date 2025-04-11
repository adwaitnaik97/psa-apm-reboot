cmake_minimum_required(VERSION 2.8.3)
project(hg_node C CXX)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)


#Include the HGuide API - creates 'hguide_api' shared library
include(${CMAKE_CURRENT_LIST_DIR}/src/HGuideAPI/CMakeLists.txt)
 
#Include Custom messages - creates 'hg_node_message_support' shared library
include(${CMAKE_CURRENT_LIST_DIR}/src/messages/CMakeLists.txt)
 
 
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


link_directories(/usr/local/lib/)

add_library(hg_node_api_to_topics src/api_to_topics.cpp)
target_link_libraries(hg_node_api_to_topics ${catkin_LIBRARIES} hguide_api hg_node_message_support)
add_dependencies(hg_node_api_to_topics hg_node_generate_messages_cpp)

## Basic functionality supporting all IMUs
add_executable(serial_publisher src/serial_publisher.cpp)
target_link_libraries(serial_publisher ${catkin_LIBRARIES} hguide_api hg_node_api_to_topics)
add_dependencies(serial_publisher hg_node_generate_messages_cpp)

add_executable(listener_example src/listener_example.cpp)
target_link_libraries(listener_example ${catkin_LIBRARIES})
add_dependencies(listener_example hg_node_generate_messages_cpp)

add_executable(save_csv src/saveCSV.cpp)
target_link_libraries(save_csv ${catkin_LIBRARIES})
add_dependencies(save_csv hg_node_generate_messages_cpp)

