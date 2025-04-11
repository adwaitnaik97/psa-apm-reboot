cmake_minimum_required(VERSION 3.0.2)
project(junction_id_publisher)
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

set(junction_id_publisher_SRC
        src/junction_id_publisher.cpp
        include/junction_id_publisher/junction_id_publisher.h
        )

add_library(junction_id_publisher_lib SHARED
        ${junction_id_publisher_SRC}
        )

target_include_directories(junction_id_publisher_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        include
        )

target_link_libraries(junction_id_publisher_lib
        ${OpenCV_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_dependencies(junction_id_publisher_lib
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(junction_id_publisher
        src/junction_id_publisher.cpp
        )
        
target_include_directories(junction_id_publisher PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        include
        )

target_link_libraries(junction_id_publisher
        junction_id_publisher_lib)

install(TARGETS junction_id_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/junction_id_publisher/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
