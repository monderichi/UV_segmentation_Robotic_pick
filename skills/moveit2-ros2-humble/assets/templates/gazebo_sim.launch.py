#!/usr/bin/env python3
"""
Gazebo Sim launch file template for MoveIt2 robot simulation.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package name - update this
    pkg_name = "my_robot_moveit_config"
    description_pkg = "my_robot_description"
    
    pkg_share = FindPackageShare(pkg_name)
    desc_pkg_share = FindPackageShare(description_pkg)
    
    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    gazebo_gui = LaunchConfiguration("gazebo_gui", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    world_file = LaunchConfiguration("world_file", default="empty.sdf")
    
    # Set Gazebo resource paths
    set_gazebo_resources = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([pkg_share, "worlds"]),
    )
    
    set_gazebo_models = SetEnvironmentVariable(
        name="IGN_GAZEBO_MODEL_PATH",
        value=PathJoinSubstitution([desc_pkg_share, "models"]),
    )
    
    # Robot description
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            desc_pkg_share,
            "urdf",
            "robots",
            "my_robot.urdf.xacro",
        ]),
    ])
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": use_sim_time,
        }],
        output="screen",
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ]),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([pkg_share, "worlds", world_file]),
                " -r",  # Run on startup
                LaunchConfiguration("gui_args"),
            ],
            "on_exit_shutdown": "false",
        }.items(),
    )
    
    # Spawn robot
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "my_robot",
            "-topic", "/robot_description",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
        output="screen",
    )
    
    # Clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )
    
    # Camera bridge (if using RealSense)
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/camera/color/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera/depth/image_rect_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_camera", default="false")),
    )
    
    # Controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )
    
    arm_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "arm_controller",
        ],
        output="screen",
    )
    
    # MoveIt move_group
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "kinematics.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "joint_limits.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "ompl_planning.yaml"]),
            PathJoinSubstitution([pkg_share, "config", "moveit_controllers.yaml"]),
            {
                "robot_description": robot_description_content,
                "robot_description_semantic": Command([
                    "cat ",
                    PathJoinSubstitution([pkg_share, "config", "my_robot.srdf"]),
                ]),
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
    )
    
    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "rviz", "moveit.rviz"]),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world_file", default_value="empty.sdf"),
        DeclareLaunchArgument("use_camera", default_value="false"),
        DeclareLaunchArgument(
            "gui_args",
            default_value="",
            description="Additional Gazebo GUI arguments",
        ),
        
        # Environment
        set_gazebo_resources,
        set_gazebo_models,
        
        # Nodes
        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        camera_bridge,
        
        # Event handlers for startup sequence
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller,
                on_exit=[move_group, rviz],
            )
        ),
    ])
