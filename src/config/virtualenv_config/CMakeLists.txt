cmake_minimum_required(VERSION 3.10.2)
project(virtualenv_config)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)

# run virtualenv_build.sh at build time
add_custom_target(
  VirtualEnv ALL
  DEPENDS aios-env
)
add_custom_command(OUTPUT aios-env
                   COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/virtualenv_build.sh
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)
catkin_package()

# run virtualenv_source.sh when workplace is sourced
catkin_add_env_hooks(50.virtualenv_source SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(FILES virtualenv_source.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
