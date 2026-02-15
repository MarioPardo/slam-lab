cmake_minimum_required(VERSION 3.15)
project(slam_core VERSION 1.0.0 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Find ZeroMQ (cppzmq header-only wrapper + zmq library)
find_package(cppzmq REQUIRED)

# Find Eigen3 (header-only linear algebra library)
find_package(Eigen3 REQUIRED NO_MODULE)


set(SOURCES
    src/main.cpp
    src/zmq_bridge.cpp
    src/odometry.cpp
    src/lidar_processor.cpp
    src/occupancy_grid.cpp
    src/icp_matcher.cpp
)


set(HEADERS
    src/zmq_bridge.h
    src/odometry.h
    src/lidar_processor.h
    src/icp_matcher.h
    src/types.h
)


add_executable(slam_subscriber ${SOURCES} ${HEADERS})


target_link_libraries(slam_subscriber 
    PRIVATE 
    cppzmq
    Eigen3::Eigen
)


target_include_directories(slam_subscriber
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


install(TARGETS slam_subscriber
    RUNTIME DESTINATION bin
)

# Test executable for ICP
add_executable(test_icp
    tests/test_icp_transform.cpp
    src/icp_matcher.cpp
)

target_link_libraries(test_icp
    PRIVATE
    Eigen3::Eigen
)

target_include_directories(test_icp
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(test_icp_full
    tests/test_icp_full.cpp
    src/icp_matcher.cpp
)

target_link_libraries(test_icp_full
    PRIVATE
    Eigen3::Eigen
)

target_include_directories(test_icp_full
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(test_scan_conversion
    tests/test_scan_conversion.cpp
    src/lidar_processor.cpp
)

target_link_libraries(test_scan_conversion
    PRIVATE
    Eigen3::Eigen
)

target_include_directories(test_scan_conversion
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
