cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_sensor_config)

find_package(catkin REQUIRED)
catkin_package(
)

install(DIRECTORY camera/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/camera)
install(DIRECTORY lidar/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lidar)
