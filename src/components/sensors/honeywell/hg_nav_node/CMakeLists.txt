cmake_minimum_required(VERSION 2.8.3)
project(hg_nav_node C CXX)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  tf
  tf2
  tf2_ros
  tf_conversions
)

#Include the HGuide API - creates 'hguide_api' shared library
include(${CMAKE_CURRENT_LIST_DIR}/src/HGuideAPI/CMakeLists.txt)

#Include Custom messages - creates 'hg_nav_node_message_support' shared library
include(${CMAKE_CURRENT_LIST_DIR}/src/messages/CMakeLists.txt)



###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
	#ftd2xx
#  INCLUDE_DIRS include
#  LIBRARIES hg_node
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime tf tf2 tf2_ros tf_conversions
#  DEPENDS system_lib
)

#link_directories(/usr/local/lib/libftd2xx.so)
link_directories(/usr/local/lib/)

add_library(api_to_topics src/api_to_topics.cpp )
target_link_libraries(api_to_topics ${catkin_LIBRARIES} hguide_api hg_nav_node_message_support)
add_dependencies(api_to_topics hg_nav_node_generate_messages_cpp)

# -Wall -Wextra -lrt -Wl -rpath /usr/local/lib -L/usr/local/lib

#add_executable(ftdi_example src/main.cpp )
#target_link_libraries(ftdi_example ${catkin_LIBRARIES} hg_parser ftd2xx)

## Navigation functionality supporting all INSs
add_executable(serial_publisher_nav src/serial_publisher.cpp)
target_link_libraries(serial_publisher_nav ${catkin_LIBRARIES} hguide_api api_to_topics)
add_dependencies(serial_publisher_nav hg_nav_node_generate_messages_cpp)

add_executable(replay_publisher_nav src/replay_publisher.cpp)
target_link_libraries(replay_publisher_nav ${catkin_LIBRARIES} hguide_api api_to_topics)
add_dependencies(replay_publisher_nav hg_nav_node_generate_messages_cpp)

add_executable(listener_example_nav src/listener_example.cpp)
target_link_libraries(listener_example_nav ${catkin_LIBRARIES})
add_dependencies(listener_example_nav hg_nav_node_generate_messages_cpp)

add_executable(n580_imu_data_nav src/n580_imu_data.cpp)
target_link_libraries(n580_imu_data_nav ${catkin_LIBRARIES})
add_dependencies(n580_imu_data_nav hg_nav_node_generate_messages_cpp)

add_executable(demo_mode_nav src/DemoMode.cpp)
target_link_libraries(demo_mode_nav ${catkin_LIBRARIES})
add_dependencies(demo_mode_nav hg_nav_node_generate_messages_cpp)

add_executable(saveCSV_nav src/saveCSV.cpp)
target_link_libraries(saveCSV_nav ${catkin_LIBRARIES} api_to_topics)
add_dependencies(saveCSV_nav hg_nav_node_generate_messages_cpp)

add_executable(vehicle_tf src/vehicle_tf.cpp)
target_link_libraries(vehicle_tf ${catkin_LIBRARIES} yaml-cpp)
