cmake_minimum_required(VERSION 3.0.2)
project(v2i_handler)
add_compile_options(-std=c++14)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(CURL REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  aifo_client
  geometry_msgs
  aios_apm_msgs
  aipe_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   std_msgs

)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENSSL_INCLUDE_DIR}
)

add_executable(v2i_handler src/v2i_handler.cpp)
add_dependencies(v2i_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(v2i_handler
   ${catkin_LIBRARIES}
   ${OPENSSL_LIBRARIES} )
target_link_libraries(${PROJECT_NAME} CURL::libcurl)
install(TARGETS v2i_handler
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)