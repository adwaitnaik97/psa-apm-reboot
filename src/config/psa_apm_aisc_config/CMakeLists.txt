cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_aisc_config)

find_package(catkin REQUIRED)
catkin_package(
)

install(DIRECTORY situation_cognition/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/situation_cognition)
