cmake_minimum_required(VERSION 3.0.2)
project(v2i_control)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        sensor_msgs
        roscpp
        rostest
        rosunit
        tf
        aipe_msgs
        aisc_msgs
        aios_apm_msgs
        airs_msgs
        aifo_client
        jsk_recognition_utils
        tf_conversions
        jsk_topic_tools
        jsk_topic_tools
        visualization_msgs
        traffic_light_common
        )

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

catkin_package(
        CATKIN_DEPENDS
        cv_bridge
        sensor_msgs
        roscpp
        tf
        aipe_msgs
        aisc_msgs
        aios_apm_msgs
        airs_msgs
        aifo_client
        jsk_recognition_utils
        tf_conversions
        jsk_topic_tools
        visualization_msgs
        traffic_light_common
)

set(v2i_control_SRC
        src/v2i_control.cpp
        src/v2i_control_node.cpp
        include/v2i_control/v2i_control.h
        include/v2i_control/v2i_control_node.h
        )

add_library(v2i_control_lib SHARED
        ${v2i_control_SRC}
        )

target_include_directories(v2i_control_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        include
        )

target_link_libraries(v2i_control_lib
        ${OpenCV_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_dependencies(v2i_control_lib
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(v2i_control
        src/v2i_control_node.cpp
        )
        
target_include_directories(v2i_control PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        include
        )

target_link_libraries(v2i_control
        v2i_control_lib)

install(TARGETS v2i_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/v2i_control/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roscpp REQUIRED)
  #catkin_add_gtest(test-v2i_control test/src/test_v2i_control.cpp)
  
  add_rostest_gtest(test-v2i_control
  test/test_v2i_control.test
  test/src/test_v2i_control.cpp
  #${v2i_control_SRC}
  )

  target_link_libraries(test-v2i_control
  ${catkin_LIBRARIES}
  v2i_control_lib)

  target_include_directories(test-v2i_control PRIVATE
        ${catkin_INCLUDE_DIRS}
        include
        )
  
endif ()
