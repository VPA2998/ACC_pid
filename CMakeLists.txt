cmake_minimum_required(VERSION 3.10)
project(ACC_Simulator LANGUAGES CXX)

# Require at least C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler warnings (optional but recommended)
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Source files
set(SRC_FILES
    main.cpp
)

# Include headers
include_directories(${CMAKE_SOURCE_DIR})

# Create executable
add_executable(acc_simulator ${SRC_FILES})

# Message on success
message(STATUS "✅ ACC Simulator ready to build")
