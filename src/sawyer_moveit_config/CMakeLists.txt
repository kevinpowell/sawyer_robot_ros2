cmake_minimum_required(VERSION 3.5)
project(sawyer_moveit_config)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY launch srdf config rviz
        DESTINATION share/sawyer_moveit_config
)


ament_package()