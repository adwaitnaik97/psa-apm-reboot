cmake_minimum_required(VERSION 3.0.2)
project(rslidar_sync)
find_package(catkin REQUIRED roscpp roslib std_msgs sensor_msgs message_filters)
add_compile_options(-std=c++14)
catkin_package()

include_directories(include
    ${catkin_INCLUDE_DIRS})

add_executable(rslidar_sync_2lidar_node src/timesync_2lidar.cpp)
target_link_libraries(rslidar_sync_2lidar_node
	${catkin_LIBRARIES}
)

add_executable(rslidar_sync_3lidar_node src/timesync_3lidar.cpp)
target_link_libraries(rslidar_sync_3lidar_node
	${catkin_LIBRARIES}
)

install(TARGETS rslidar_sync_3lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rslidar_sync_2lidar_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz_cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
