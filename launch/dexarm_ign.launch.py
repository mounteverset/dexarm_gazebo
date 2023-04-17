#!/usr/bin/python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import xacro
import yaml


# Things that get launched in this file:
# Robot State Publisher
# Gazebo
# Gazebo Spawner
# MoveIt
# RViz
# Controller Manager
# Joint State Broadcaster
# Joint Trajectory Controller

# Things that are needed in this file:
# Robot Description
# Robot Description Config
# Rviz Config
# 

LAUNCH_ARGS = [
    DeclareLaunchArgument('robot_name', default_value='dexarm', description='Robot name'),
    DeclareLaunchArgument('x', default_value='0.0', description='The x-coordinate for the robot'),
    DeclareLaunchArgument('y', default_value='0.0', description='The y-coordinate for the robot'),
    DeclareLaunchArgument('z', default_value='0.0', description='The x-coordinate for the robot'),
    DeclareLaunchArgument('rviz', default_value='false', description='Start rviz.'),
]

def generate_launch_description():
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    rviz = LaunchConfiguration('rviz')
    robot_name = LaunchConfiguration('robot_name')

    
    
    # Description file package:
    # dexarm_description_path = os.path.join(get_package_share_directory('dexarm_description'))
    # # Dexarm urdf file path:
    # xacro_file = os.path.join(dexarm_description_path, 'urdf', 'dexarm.xacro', ' ', '')
    # doc = xacro.process_file(xacro_file, )
    # robot_description = doc.toprettyxml(indent='  ')

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([get_package_share_directory('dexarm_description'), "urdf", 'dexarm.xacro']),
            " ",
            "is_simulation:=",
            "True",
            " ",
            "use_fake_hardware:=",
            "False",
            " ",
            "fake_sensor_commands:=",
            "False",
        ]
    )

    # DECLARE Gazebo LAUNCH file:
    gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r --headless-rendering empty.sdf'}.items(),
    )

    # RSP 
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": robot_description},
                    {"use_sim_time": True}]
    )

    # Spawn dexarm
    spawn_entity_cmd = Node(package='ros_gz_sim', 
                            executable='create',
                            arguments=['-topic', 'robot_description',
                                        '-name', robot_name,
                                        '-x', x,
                                        '-y', y,
                                        '-z', z],
                            output='screen')

    # Gz - ROS Bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (IGN -> ROS2)
            {'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'},
            # Joint states (IGN -> ROS2)
            # {'/world/empty/model/dexarm_model/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'},
            ],
        # remappings=[('/world/empty/model/dexarm_model/joint_state', 'joint_states')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("dexarm_moveit_config"), "launch", "moveit_rviz.launch.py")])
    )

    robot_description_controller_launch = os.path.join(get_package_share_directory("dexarm_description"), "launch", "controllers.launch.py")
    
    # # Why timed? Why not just include the launch file?
    # robot_controllers_cmd = TimerAction(
    #     period=10.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([robot_description_controller_launch]),
    #             launch_arguments=[
    #                 ('use_sim_time', 'true'),
    #                 ('use_fake_hardware', 'false'),
    #                 ('fake_sensor_commands', 'false'),
    #             ]
    #         )]
    # )

    arm_controller_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        parameters=[{'use_sim_time': True}],
    )

    joint_state_broadcaster_spawner_cmd = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
    )

    srdf_path = os.path.join(get_package_share_directory("dexarm_description"), "config", "dexarm.srdf")
    with open(srdf_path, 'r') as f:
        semantic_content = f.read()

    moveit_controller_path = os.path.join(get_package_share_directory("dexarm_description"), "config", "moveit_controllers.yaml")

    # Load the YAML file's contents
    with open(moveit_controller_path, 'r') as file:
        moveit_controller_params = yaml.safe_load(file)

# Merge the dictionaries
    parameters_dict = {
        "robot_description": robot_description,
        "robot_description_semantic": semantic_content,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        # "capabilities": ParameterValue(
        #     LaunchConfiguration("capabilities"), value_type=str
        # ),
        # "disable_capabilities": ParameterValue(
        #     LaunchConfiguration("disable_capabilities"), value_type=str
        # ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": True,
    }

    # Combine the two dictionaries
    parameters_dict.update(moveit_controller_params)

    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[parameters_dict]
    )

    # move_group_cmd = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         {
    #         "robot_description": robot_description,
    #         "robot_description_semantic": semantic_content,
    #         "publish_robot_description_semantic": True,
    #         "allow_trajectory_execution": True,
    #         # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
    #         # "capabilities": ParameterValue(
    #         #     LaunchConfiguration("capabilities"), value_type=str
    #         # ),
    #         # "disable_capabilities": ParameterValue(
    #         #     LaunchConfiguration("disable_capabilities"), value_type=str
    #         # ),
    #         # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
    #         "publish_planning_scene": True,
    #         "publish_geometry_updates": True,
    #         "publish_state_updates": True,
    #         "publish_transforms_updates": True,
    #         "monitor_dynamics": False,
    #         "use_sim_time": True,
    #         },
    #         moveit_controller_path
    #     ]
    # )

    ld = LaunchDescription(LAUNCH_ARGS)
    # ld.add_action(x_arg)
    # ld.add_action(y_arg)
    # ld.add_action(z_arg)
    # ld.add_action(robot_name_arg)
    # ld.add_action(rviz_arg)
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(arm_controller_spawner_cmd)
    ld.add_action(joint_state_broadcaster_spawner_cmd)
    ld.add_action(move_group_cmd)
    ld.add_action(rviz_cmd)

    return ld

