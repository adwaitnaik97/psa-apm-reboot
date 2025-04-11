cmake_minimum_required(VERSION 3.0.2)
project(novatel_oem7_launcher)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  
)


catkin_package(
  # LIBRARIES novatel_oem7_launcher
  CATKIN_DEPENDS roscpp rospy std_msgs 
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
