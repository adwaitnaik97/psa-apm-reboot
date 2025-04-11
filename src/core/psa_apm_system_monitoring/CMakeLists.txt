cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_system_monitoring)

add_compile_options(-std=c++14)


set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    ros_type_introspection
    topic_tools
    )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ros_type_introspection
  topic_tools
  message_filters
  apm_diagnostic_msgs
  diagnostic_msgs
  aios_apm_msgs
 )
find_package(Boost)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  #include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)




add_executable(core_system_monitoring src/core_system_monitoring.cpp)
add_dependencies(core_system_monitoring ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(core_system_monitoring
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES})



install(TARGETS core_system_monitoring
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


