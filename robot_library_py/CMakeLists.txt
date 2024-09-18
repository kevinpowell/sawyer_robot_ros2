cmake_minimum_required(VERSION 3.8)
project(robot_library_py)

set(CMAKE_CXX_STANDARD 17)
find_package (Python3 COMPONENTS Interpreter Development)

if(PYTHON3_FOUND)
    include_directories(${PYTHON3_INCLUDE_DIRS})
    message("Using default python version: " ${PYTHON3_VERSION})
endif()

# add binding library/module
add_subdirectory(parse_urdf)
find_package(ament_cmake REQUIRED)
ament_package()
