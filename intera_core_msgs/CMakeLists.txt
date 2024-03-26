cmake_minimum_required(VERSION 3.5)
project(intera_core_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/AnalogIOState.msg
    msg/AnalogIOStates.msg
    msg/AnalogOutputCommand.msg
    msg/RobotAssemblyState.msg
    msg/CameraControl.msg
    msg/CameraSettings.msg
    msg/CollisionAvoidanceState.msg
    msg/CollisionDetectionState.msg
    msg/DigitalIOState.msg
    msg/DigitalIOStates.msg
    msg/DigitalOutputCommand.msg
    msg/EndpointNamesArray.msg
    msg/EndpointState.msg
    msg/EndpointStates.msg
    msg/HeadPanCommand.msg
    msg/HeadState.msg
    msg/HomingCommand.msg
    msg/HomingState.msg
    msg/InteractionControlCommand.msg
    msg/InteractionControlState.msg
    msg/IOComponentCommand.msg
    msg/IOComponentConfiguration.msg
    msg/IOComponentStatus.msg
    msg/IODataStatus.msg
    msg/IODeviceConfiguration.msg
    msg/IODeviceStatus.msg
    msg/IONodeConfiguration.msg
    msg/IONodeStatus.msg
    msg/IOStatus.msg
    msg/JointCommand.msg
    msg/JointLimits.msg
    msg/NavigatorState.msg
    msg/NavigatorStates.msg
    msg/SEAJointState.msg
    msg/URDFConfiguration.msg
  )

rosidl_generate_interfaces(intera_core_msgs
    ${msg_files}
    DEPENDENCIES
    action_msgs
    std_msgs
    builtin_interfaces
    geometry_msgs
    )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
