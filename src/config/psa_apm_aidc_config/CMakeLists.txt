cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_aidc_config)

find_package(catkin REQUIRED)
catkin_package(
)

install(DIRECTORY traffic_light_control/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/psa_apm_core_config/traffic_light_control)
