cmake_minimum_required(VERSION 3.5)
project(intera_tools_description)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY meshes urdf
        DESTINATION share/intera_tools_description
)


ament_package()
