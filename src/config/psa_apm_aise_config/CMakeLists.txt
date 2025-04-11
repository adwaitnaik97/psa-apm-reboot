cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_aise_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)