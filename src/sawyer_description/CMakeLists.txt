cmake_minimum_required(VERSION 3.5)
project(sawyer_description)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY launch meshes urdf  rviz
        DESTINATION share/sawyer_description
)


ament_package()