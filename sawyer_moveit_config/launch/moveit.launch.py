import os

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_content}],
    )

    # robot_description_semantic_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("sawyer_moveit_config"),
    #                 "srdf",
    #                 "full_sawyer.srdf.xacro",
    #             ]
    #         ),
    #     ]
    # )

    PATH_TO_SRDF = os.path.join(get_package_share_directory('sawyer_moveit_config'), "srdf",
                                "full_sawyer.srdf.xacro")
    with open(PATH_TO_SRDF, 'r') as f:
        robot_description_semantic_content = f.read()

    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                           output='screen',
                           parameters=[{
                               'robot_description': robot_description_content,
                               'robot_description_semantic': robot_description_semantic_content,
                               # More params
                           }],
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
        parameters=[{
            'robot_description_semantic': robot_description_semantic_content,
        }]
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node,
        move_group_node,
    ]

    return LaunchDescription(nodes_to_start)


generate_launch_description()
