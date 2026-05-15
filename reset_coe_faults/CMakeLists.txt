cmake_minimum_required(VERSION 3.16)
project(reset_coe_faults LANGUAGES CXX)

set(reset_coe_faults_MAJOR_VERSION 0)
set(reset_coe_faults_MINOR_VERSION 0)
set(reset_coe_faults_PATCH_VERSION 0)
set(reset_coe_faults_VERSION
  ${reset_coe_faults_MAJOR_VERSION}.${reset_coe_faults_MINOR_VERSION}.${reset_coe_faults_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(reset_coe_faults
    src/reset_coe_faults.cpp
)

target_compile_options(reset_coe_faults PRIVATE
    -Wall
    -Wextra
    -O2
)

# Common IgH/EtherLab install locations.
find_path(ETHERCAT_INCLUDE_DIR
    NAMES ecrt.h
    PATHS
        /usr/include
        /usr/local/include
        /usr/local/etherlab/include
)

find_library(ETHERCAT_LIBRARY
    NAMES ethercat
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        /usr/local/etherlab/lib
)

if(NOT ETHERCAT_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find ecrt.h. Install IgH EtherCAT development headers or set ETHERCAT_INCLUDE_DIR.")
endif()

if(NOT ETHERCAT_LIBRARY)
    message(FATAL_ERROR "Could not find libethercat. Install IgH EtherCAT library or set ETHERCAT_LIBRARY.")
endif()

message(STATUS "Found ecrt.h: ${ETHERCAT_INCLUDE_DIR}")
message(STATUS "Found libethercat: ${ETHERCAT_LIBRARY}")

target_include_directories(reset_coe_faults PRIVATE
    ${ETHERCAT_INCLUDE_DIR}
)

target_link_libraries(reset_coe_faults
    ${ETHERCAT_LIBRARY}
    rt
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
