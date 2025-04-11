cmake_minimum_required(VERSION 3.0.2)
project(tl_vision_control)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        sensor_msgs
        image_transport
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
        eigen_conversions
        tf_conversions
        jsk_topic_tools
        image_geometry
        jsk_topic_tools
        visualization_msgs
        traffic_light_common
        aipe_utils
        )

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARIES})

catkin_package(
        CATKIN_DEPENDS
        cv_bridge
        sensor_msgs
        image_transport
        roscpp
        tf
        aipe_msgs
        aisc_msgs
        aios_apm_msgs
        airs_msgs
        aifo_client
        jsk_recognition_utils
        eigen_conversions
        tf_conversions
        jsk_topic_tools
        image_geometry
        jsk_topic_tools
        visualization_msgs
        traffic_light_common
        aipe_utils
)

#fusion Library
set(tl_vision_control_SRC
        src/tl_vision_control.cpp
	src/junction_state_estimator.cpp
        src/CAS_state_estimator.cpp
        src/side_tl_detector.cpp
        src/PSA_exceptional_cases.cpp
        include/tl_vision_control/tl_vision_control.h
	include/tl_vision_control/junction_state_estimator.h
        include/tl_vision_control/CAS_state_estimator.h
        include/tl_vision_control/side_tl_detector.h
        include/tl_vision_control/PSA_exceptional_cases.h
        )

add_library(tl_vision_control_lib SHARED
        ${tl_vision_control_SRC}
        )

if (OPENMP_FOUND)
    set_target_properties(tl_vision_control_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(tl_vision_control_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${OPENGL_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        ${aipe_utils_INCLUDE_DIRS}
        include
        )

target_link_libraries(tl_vision_control_lib
        ${OpenCV_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${OPENGL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
        )

add_dependencies(tl_vision_control_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(tl_vision_control
        src/tl_vision_control.cpp
        )
target_include_directories(tl_vision_control PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${traffic_light_common_INCLUDE_DIRS}
        ${aipe_utils_INCLUDE_DIRS}
        include
        )

target_link_libraries(tl_vision_control
        tl_vision_control_lib)

install(TARGETS tl_vision_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/tl_vision_control/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roscpp REQUIRED)
  #catkin_add_gtest(test-tl_vision_control test/src/test_tl_vision_control.cpp)
  
  add_rostest_gtest(test-tl_vision_control
  test/test_tl_vision_control.test
  test/src/test_tl_vision_control.cpp
  #${tl_vision_control_SRC}
  )

  target_link_libraries(test-tl_vision_control
  ${catkin_LIBRARIES}
  tl_vision_control_lib)

  target_include_directories(test-tl_vision_control PRIVATE
        ${catkin_INCLUDE_DIRS}
        include
        )
  
endif ()
