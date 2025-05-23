#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('moveit_ur_config')

    # 1) Robot description (URDF via xacro)
    xacro_path = os.path.join(pkg_share, 'config', 'ur5.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # 2) Semantic SRDF
    srdf_path = os.path.join(pkg_share, 'config', 'ur5.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = f.read()

    # 3) Paths to your YAML configs
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')
    servo_yaml       = os.path.join(pkg_share, 'config', 'servo.yaml')
    kinematics_yaml  = os.path.join(pkg_share, 'config', 'kinematics.yaml')

    # 4) RSP
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 5) ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_yaml
        ]
    )

    # 6) Spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen'
    )

    # 7) MoveIt Servo core node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic},
            ParameterFile(servo_yaml,      allow_substs=True),
            ParameterFile(kinematics_yaml, allow_substs=True)
        ]
    )

    # 8) Joy node (publishes /joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    # 9) Composable Joy-to-Servo publisher
    joy_to_servo_comp = ComposableNode(
        package='moveit_servo',
        plugin='moveit_servo::JoyToServoPub',
        name='joy_to_servo',
        parameters=[ParameterFile(servo_yaml, allow_substs=True)]
    )
    joy_container = ComposableNodeContainer(
        name='joy_servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[joy_to_servo_comp],
        output='screen'
    )

    # 10) RViz
    rviz_config = os.path.join(pkg_share, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': robot_description_semantic}
        ]
    )

    return LaunchDescription([
        rsp_node,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        servo_node,
        joy_node,
        joy_container,
        rviz_node
    ])
