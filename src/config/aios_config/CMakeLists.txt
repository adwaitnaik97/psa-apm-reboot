cmake_minimum_required(VERSION 3.0.2)
project(aios_config)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

install(FILES PSA_APM_8380_aios_config.bash PSA_APM_8381_aios_config.bash _aios_config.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_env_hooks(50.aios_config SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY tf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tf)


install(DIRECTORY configuration_data/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configuration_data)
