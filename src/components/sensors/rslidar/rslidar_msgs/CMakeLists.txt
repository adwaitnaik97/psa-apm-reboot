cmake_minimum_required(VERSION 3.0.2)
project(rslidar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_compile_options(-std=c++14)
add_message_files(
  DIRECTORY msg
  FILES
  rslidarPacket.msg
  rslidarScan.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
