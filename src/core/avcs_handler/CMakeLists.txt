cmake_minimum_required(VERSION 3.0.2)
project(avcs_handler)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  aios_apm_msgs
  aidc_msgs
  gps_common
  nlohmann
  aide_apm_msgs
  nav_msgs
  pcl_ros
)

find_package( Eigen3)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs aios_apm_msgs aide_apm_msgs nav_msgs aidc_msgs gps_common nlohmann)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(avcs_handler src/avcs_handler.cpp)
add_dependencies(avcs_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(avcs_handler
   ${catkin_LIBRARIES})

install(TARGETS avcs_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
