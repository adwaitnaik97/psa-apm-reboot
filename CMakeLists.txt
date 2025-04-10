# cmake min version 3.25.2
cmake_minimum_required(VERSION 3.25.2)

# Set the C++ standard to C++17 and make it required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# vcpkg dependencies
set(VCPKG_MANIFEST_INSTALL ON CACHE BOOL "Install vcpkg dependencies")

# Set the version and package name
set(PROJECT_VERSION 0.0.1)
set(PROJECT_NAME psa_apm_reboot)

include(cmake/vcpkg/vcpkg_use_submodule.cmake)

init_vcpkg()

# set vcpkg options
set(VCPKG_BOOTSTRAP_OPTIONS
##    --triplet x64-windows-static
##    --overlay-ports ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_ports
##    --overlay-triplets ${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_triplets
vcpkg_use_submodule(SUBMODULE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg" CACHE BOOL "Use vcpkg submodule")
)

# unable ccache if it is disabled
unset(ENV{CCACHE_DISABLE})

# set project details
project(${PROJECT_NAME}
  VERSION "${PROJECT_VERSION}"
  LANGUAGES C CXX
  )

# add sub-directories
add_subdirectory(src/)

