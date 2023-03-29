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

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='dexarm', description='Robot name')
    x_arg =  DeclareLaunchArgument('x', default_value='0.0', description='The x-coordinate for the robot')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='The y-coordinate for the robot')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='The x-coordinate for the robot')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Start rviz.')
    

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

    # doc = xacro.parse(open(xacro_file))

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

    # Outputs error
    # rviz2_cmd = Node(
    #     IfCondition(rviz),
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[
    #         {'use_sim_time': True}
    #     ],
    #     output='screen')

    rviz_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("dexarm_moveit_config"), "launch", "moveit_rviz.launch.py")])
    )
    
    # Delay launch of the controllers until after Ignition has launched and the model
    # has spawned in Ignition because the Ignition controller plugins don't become available
    # until the model has spawned

    robot_description_controller_launch = os.path.join(get_package_share_directory("dexarm_moveit_config"), "launch", "controllers.launch.py")

    robot_controllers_cmd = TimerAction(
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

    srdf_path = os.path.join(get_package_share_directory("dexarm_moveit_config"), "config", "dexarm.srdf")

    with open(srdf_path, 'r') as f:
        semantic_content = f.read()

    move_group_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {
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
            "use_sim_time": True
            }
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(rviz_arg)
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(robot_controllers_cmd)
    ld.add_action(move_group_cmd)
    


    # ***** RETURN LAUNCH DESCRIPTION ***** #
    # return LaunchDescription([
    #     bridge_cmd,
    #     # move_group_cmd,
    #     # rviz_cmd,
    #     # moveit_cmd,
    #     gazebo_cmd, 
    #     robot_state_publisher_cmd,
    #     spawn_entity_cmd,
    #     rviz2
    # ])

    return ld