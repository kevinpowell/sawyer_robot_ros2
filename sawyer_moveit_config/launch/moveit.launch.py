import os

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.execute_local import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    real = LaunchConfiguration('real_bot')
    real_bot_larg = DeclareLaunchArgument('real_bot', default_value='True')
    if real=='True': #note gumpy string compare
        robot_joint_states_topic = 'robot/joint_states'
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("sawyer_description"),
                        "urdf",
                        "full_sawyer.urdf.xacro",
                    ]
                ),
            ]
        )
    else:
        robot_joint_states_topic = 'joint_states'
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("sawyer_description"),
                        "urdf",
                        "sim_full_sawyer.urdf.xacro",
                    ]
                ),
            ]
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_content}],
    )

    srdf_path = os.path.join(get_package_share_directory('sawyer_moveit_config'), "srdf",
                             "full_sawyer.srdf.xacro")
    with open(srdf_path, 'r') as f:
        robot_description_semantic_content = f.read()

    kinematics_path = os.path.join(get_package_share_directory("sawyer_moveit_config"),
                                   "config",
                                   "sawyer_parameters.yaml")

    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
            "collision_detection": "neural",
        }
    }
    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=[{'robot_description': robot_description_content},
                                       {'robot_description_semantic': robot_description_semantic_content},
                                       {
                                           'moveit_controller_manager': 'moveit_ros_control_interface/Ros2ControlMultiManager'},
                                       {'moveit_manage_controllers': True},
                                       {'ros_control_namespace': '/'},
                                       {'planning_scene_monitor_options.joint_state_topic': robot_joint_states_topic},
                                       ompl_planning_pipeline_config,
                                       kinematics_path,
                                       ],
                           )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("sawyer_moveit_config"), "rviz", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            {'robot_description_semantic': robot_description_semantic_content},
            kinematics_path,
        ]
    )

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("sawyer_moveit_config"), "config", "ros2_controllers.yaml"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description_content},
            initial_joint_controllers,
        ],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--stopped"],
    )

    nodes_to_start = [
        real_bot_larg,
        control_node,
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
        joint_trajectory_controller_spawner
    ]

    return LaunchDescription(nodes_to_start)
