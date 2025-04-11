cmake_minimum_required(VERSION 3.0.2)
project(honeywell_rad_to_deg)
add_compile_options(-std=c++14)
find_package(	catkin REQUIRED
		roscpp
		std_msgs
		airs_msgs
		message_generation)

include_directories(
${catkin_INCLUDE_DIRS}
)



generate_messages(
	   	DEPENDENCIES
   		std_msgs
)

add_executable(honeywell_rad_to_deg src/honeywell_rad_to_deg.cpp)
add_dependencies(honeywell_rad_to_deg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(honeywell_rad_to_deg
   ${catkin_LIBRARIES})



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
