cmake_minimum_required(VERSION 3.5)
project(sawyer_robot_driver)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcpputils REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(intera_core_msgs REQUIRED)

add_library(
    sawyer_robot_driver
    SHARED
    src/sawyer_robot_driver.cpp
)

target_include_directories(
    sawyer_robot_driver
    PUBLIC
    include
)

ament_target_dependencies(
    sawyer_robot_driver
    hardware_interface
    intera_core_msgs
    pluginlib
    rcpputils
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(sawyer_robot_driver PRIVATE "SAWYER_ROBOT_DRIVER_DLL")

pluginlib_export_plugin_description_file(hardware_interface sawyer_robot_driver_plugin_description.xml)

add_library(
    sim_sawyer_robot_driver
    SHARED
    src/sim_sawyer_robot_driver.cpp
)

target_include_directories(
    sim_sawyer_robot_driver
    PUBLIC
    include
)

ament_target_dependencies(
    sim_sawyer_robot_driver
    hardware_interface
    intera_core_msgs
    pluginlib
    rcpputils
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(sim_sawyer_robot_driver PRIVATE "SIM_SAWYER_ROBOT_DRIVER_DLL")

pluginlib_export_plugin_description_file(hardware_interface sim_sawyer_robot_driver_plugin_description.xml)




install(
    DIRECTORY include
    DESTINATION include
)

install(
    TARGETS sawyer_robot_driver sim_sawyer_robot_driver
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_package()
