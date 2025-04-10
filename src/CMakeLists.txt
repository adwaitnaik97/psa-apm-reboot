cmake_minimum_required(VERSION 3.25.2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(PSA_APM_REBOOT psa_apm_reboot)

# add executable
add_executable(${PSA_APM_REBOOT} main.cpp)

# find dependencies
find_package(Boost REQUIRED)
find_package(spdlog CONFIG REQUIRED)

target_compile_features(${PSA_APM_REBOOT} PRIVATE cxx_std_17)

# link libraries to the dependecies
target_link_libraries(${PSA_APM_REBOOT}
    PRIVATE
    spdlog::spdlog
    Boost::boost
)