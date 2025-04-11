cmake_minimum_required(VERSION 3.0.2)
project(psa_apm_aipe_config)

find_package(catkin REQUIRED)
catkin_package(
)

install(DIRECTORY cfg/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/psa_apm_aipe_config/cfg)
