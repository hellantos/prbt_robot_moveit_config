from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
    

def generate_launch_description():

    slave_config = PathJoinSubstitution(
        [FindPackageShare("prbt_robot"), "config/prbt", "prbt_0_1.dcf"]
    )

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )
    slave_node_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "3",
            "node_name": "prbt_node_1",
            "slave_config": slave_config,
            }.items(),
    )
    slave_node_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "4",
            "node_name": "prbt_node_2",
            "slave_config": slave_config,
            }.items(),
    )
    slave_node_3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "5",
            "node_name": "prbt_node_3",
            "slave_config": slave_config,
            }.items(),
    )
    slave_node_4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "6",
            "node_name": "prbt_node_4",
            "slave_config": slave_config,
            }.items(),
    )
    slave_node_5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "7",
            "node_name": "prbt_node_5",
            "slave_config": slave_config,
            }.items(),
    )
    slave_node_6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slave_launch),
        launch_arguments={
            "node_id": "8",
            "node_name": "prbt_node_6",
            "slave_config": slave_config,
            }.items(),
    )
    demo_launch = PathJoinSubstitution(
        [FindPackageShare("prbt_robot_moveit_config"), "launch", "demo.launch.py"]
    )
    nodes_to_start = [
        slave_node_1,
        slave_node_2,
        slave_node_3,
        slave_node_4,
        slave_node_5,
        slave_node_6,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments=[
                ("use_rviz", "true"),
            ]
        )
    ]

    return LaunchDescription(nodes_to_start)