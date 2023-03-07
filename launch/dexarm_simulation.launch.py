#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import xacro
import yaml

DeclareLaunchArgument('robot_name', default_value='dexarm', description='Robot name')
DeclareLaunchArgument('x', default_value='0.0', description='The x-coordinate for the robot')
DeclareLaunchArgument('y', default_value='0.0', description='The y-coordinate for the robot')
DeclareLaunchArgument('z', default_value='0.0', description='The x-coordinate for the robot')
DeclareLaunchArgument('rviz', default_value='false', description='Start rviz.'),

def generate_launch_description():
    
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    robot_name = LaunchConfiguration('robot_name')
    rviz = LaunchConfiguration('rviz')

    # DECLARE Gazebo LAUNCH file:
    gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r empty.sdf'}.items(),
            )

    # Description file package:
    dexarm_description_path = os.path.join(get_package_share_directory('dexarm_description'))

    # Dexarm urdf file path:
    xacro_file = os.path.join(dexarm_description_path, 'urdf', 'dexarm.xacro')

    #doc = xacro.parse(open(xacro_file))

    doc = xacro.process_file(xacro_file)

    robot_description = doc.toprettyxml(indent='  ')

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
            {'/world/empty/model/dexarm_model/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model'},],
        remappings=[('/world/empty/model/dexarm_model/joint_state', 'joint_states')],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Rviz2
    rviz_config = os.path.join(get_package_share_directory('dexarm_moveit_config'), 'config', 'moveit.rviz')
    rviz2 = Node(
        IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen')
    
    # Delay launch of the controllers until after Ignition has launched and the model
    # has spawned in Ignition because the Ignition controller plugins don't become available
    # until the model has spawned

    robot_controllers = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([robot_description_controller_launch]),
                launch_arguments=[
                    ('use_sim_time', 'true'),
                    ('use_fake_hardware', 'false'),
                    ('fake_sensor_commands', 'false'),
                ]
            )]
    )

    #Launch Moveit generated launch file
    # That includes
    #  * static_virtual_joint_tfs
    #  * robot_state_publisher
    #  * move_group
    #  * moveit_rviz
    #  * warehouse_db (optional)
    #  * ros2_control_node + controller spawners

    # moveit_cmd = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory("dexarm_moveit_config"), "launch", "demo.launch.py")]
    #             ))

    # move_group_cmd = IncludeLaunchDescription(
    #                     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("dexarm_moveit_config"), "launch", "move_group.launch.py")])
    # )

    # move_group_cmd = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         {"publish_robot_description_semantic": True,
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
    #         "use_sim_time": True
    #         }
    #     ]
    # )

    # rviz_cmd = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("dexarm_moveit_config"), "launch", "moveit_rviz.launch.py")])
    # )

    #Launch Gazebo

    # gazebo_world_file = os.path.join(
    #     get_package_share_directory('dexarm_gazebo'),
    #     'worlds',
    #     'empty.world')
    
    


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        bridge_cmd,
        # move_group_cmd,
        # rviz_cmd,
        # moveit_cmd,
        gazebo_cmd, 
        robot_state_publisher_cmd,
        spawn_entity_cmd,
        rviz
    ])