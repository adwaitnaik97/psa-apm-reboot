cmake_minimum_required(VERSION 3.0.2)
project(remote_handler)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  aios_apm_msgs
  aidc_msgs
  visualization_msgs
  nlohmann
  pcl_ros
  pcl_conversions  
  tf2
  tf2_ros
  aios_lib
  tf2_geometry_msgs
 
)

find_package( Eigen3)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs aios_apm_msgs visualization_msgs sensor_msgs nlohmann pcl_ros pcl_conversions tf2 tf2_ros aios_lib)
 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(publish_pcd src/publish_pcd.cpp)
add_dependencies(publish_pcd ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_pcd
   ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(publish_marker src/publish_marker.cpp)
add_dependencies(publish_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_marker ${catkin_LIBRARIES})

add_executable(vehicle_pose src/vehicle_pose.cpp)
add_dependencies(vehicle_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_pose ${catkin_LIBRARIES})


add_executable(remote_handler src/remote_handler.cpp)
add_dependencies(remote_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_handler
   ${catkin_LIBRARIES})




install(TARGETS remote_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
