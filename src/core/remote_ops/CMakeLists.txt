cmake_minimum_required(VERSION 3.0.2)
project(remote_ops)

add_compile_options(-std=c++14)
#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake-modules)
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  qt_build
  roslib
  aios_lib 
  nlohmann
)

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake-modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt5 COMPONENTS Core Gui Widgets)

find_package(CapnProto CONFIG QUIET)

find_package(PahoMqttCpp QUIET)


if (PahoMqttCpp_FOUND AND CapnProto_FOUND)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES remote_ops
 CATKIN_DEPENDS roscpp std_msgs roslib nlohmann aios_lib
# DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${CAPNP_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msg
 )

set( QT_SOURCES
    src/main.cpp
    src/remote_console.cpp
    include/remote_ops/remote_console.h
    )



#Capnp msgs

add_definitions(${CAPNP_DEFINITIONS})
capnp_generate_cpp(CAPNP_SOURCES CAPNP_HEADERS
                    msg/message.capnp
                    msg/linkup.capnp
                    msg/breaklink.capnp
                    msg/teardown.capnp
                    msg/pcd.capnp
                    msg/transform.capnp
                    msg/ego_state.capnp
                    msg/adjust_position_by_distance_cmd.capnp
                    msg/emergency_brake_cmd.capnp
                    msg/hazard_light_cmd.capnp
                    msg/head_light_cmd.capnp
                    msg/horn_cmd.capnp
                    msg/marker_array.capnp
                    msg/marker.capnp
                    msg/nudge_forward_cmd.capnp
                    msg/ops_response.capnp
                    msg/path.capnp
                    msg/pause_resume_cmd.capnp
                    msg/polygon.capnp
                    msg/precedence_override_cmd.capnp
                    msg/relocalization_cmd.capnp
                    msg/remote_ops_cmd.capnp
                    msg/remote_ops_init.capnp
                    msg/remote_ops_request.capnp
                    msg/set_destination_cmd.capnp
                    msg/set_non_yard_destination_cmd.capnp
                    msg/traffic_light_override_cmd.capnp
                    msg/trajectory_override_cmd.capnp
                    msg/speed_limit_cmd.capnp
                    msg/manual_reroute_cmd.capnp
                    msg/ping.capnp
                    msg/traffic_light.capnp
                    msg/vehicle_control_mode.capnp
                    msg/indicator_light_cmd.capnp
                    msg/gear_cmd.capnp
                    msg/fog_light_cmd.capnp
                    msg/ignition_cmd.capnp
                    msg/working_light_cmd.capnp
                    msg/ac_control_cmd.capnp
                    msg/complete_job_cmd.capnp
                    msg/weather_state_cmd.capnp
                    msg/light_state_cmd.capnp
                 )


add_library(remote_ops src/remote_ops.cpp ${CAPNP_SOURCES})
add_dependencies(remote_ops ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(remote_ops ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3 CapnProto::capnp)

add_executable(remote_mqtt_client src/remote_mqtt_client.cpp)
add_dependencies(remote_mqtt_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(remote_mqtt_client
    ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3 remote_ops)

add_executable(mytest src/mytest.cpp)
add_dependencies(mytest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(mytest
    ${catkin_LIBRARIES} PahoMqttCpp::paho-mqttpp3)


add_executable(remote_console  ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(remote_console ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(remote_console ${catkin_LIBRARIES} 
                      Qt5::Core
                      Qt5::Widgets
                      remote_ops)

endif()                      
