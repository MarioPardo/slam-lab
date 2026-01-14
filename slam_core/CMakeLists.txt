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


set(SOURCES
    src/main.cpp
    src/zmq_bridge.cpp
    src/odometry.cpp
)


set(HEADERS
    src/zmq_bridge.h
    src/odometry.h
    src/types.h
)


add_executable(slam_subscriber ${SOURCES} ${HEADERS})


target_link_libraries(slam_subscriber 
    PRIVATE 
    cppzmq
)


target_include_directories(slam_subscriber
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)


install(TARGETS slam_subscriber
    RUNTIME DESTINATION bin
)

# Print configuration
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
