# MoveIt2 Launch File Examples

Complete launch file templates for common MoveIt2 integration scenarios.

## Table of Contents

1. [RViz-Only Demo Launch](#rviz-only-demo-launch)
2. [Gazebo Classic + MoveIt Launch](#gazebo-classic--moveit-launch)
3. [Gazebo Sim + MoveIt Launch](#gazebo-sim--moveit-launch)
4. [Real Hardware Launch](#real-hardware-launch)
5. [Testing/Debugging Launch](#testingdebugging-launch)

---

## RViz-Only Demo Launch

Simplest setup for testing motion planning without simulation.

```python
#!/usr/bin/env python3
"""
RViz-only MoveIt demo launch file.

Usage:
    ros2 launch pkg demo.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package path
    pkg_share = get_package_share_directory('pkg')
    
    # Declare arguments
    declare_args = [
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'config', 'moveit.rviz'),
            description='RViz configuration file'
        ),
    ]
    
    rviz_config = LaunchConfiguration('rviz_config')
    
    # ============================================================
    # Robot Description (XACRO)
    # ============================================================
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # ============================================================
    # SRDF
    # ============================================================
    srdf_path = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # ============================================================
    # Configuration Files
    # ============================================================
    
    # Kinematics
    kinematics_path = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': 
                                     kinematics.get('robot_description_kinematics', {})}
    
    # Joint limits
    joint_limits_path = os.path.join(pkg_share, 'config', 'joint_limits.yaml')
    with open(joint_limits_path, 'r') as f:
        joint_limits = yaml.safe_load(f)
    robot_description_planning = {'robot_description_planning': 
                                   joint_limits.get('joint_limits', {})}
    
    # OMPL planning
    ompl_path = os.path.join(pkg_share, 'config', 'ompl_planning.yaml')
    with open(ompl_path, 'r') as f:
        ompl_config = yaml.safe_load(f)
    
    ompl_planning_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline['move_group'].update(ompl_config)
    
    # MoveIt controllers (mock for demo)
    moveit_controllers = {
        'moveit_simple_controller_manager': {},
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # ============================================================
    # Nodes
    # ============================================================
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )
    
    # Joint State Publisher (GUI for testing)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )
    
    # Move Group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
        ],
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )
    
    return LaunchDescription(declare_args + [
        robot_state_publisher,
        joint_state_publisher,
        move_group,
        rviz,
    ])
```

---

## Gazebo Classic + MoveIt Launch

```python
#!/usr/bin/env python3
"""
Gazebo Classic + MoveIt2 integration launch.

Usage:
    ros2 launch pkg gazebo_moveit.launch.py
    ros2 launch pkg gazebo_moveit.launch.py gazebo_gui:=false
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
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
    pkg_share = get_package_share_directory('pkg')
    
    # Arguments
    declare_args = [
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('world', default_value='empty.world'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.0'),
    ]
    
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    
    # Robot description with Gazebo enabled
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
        ' use_gazebo:=true',
    ])
    
    robot_description_param = {'robot_description': robot_description}
    
    # Load SRDF and configs (same as demo)
    srdf_path = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    kinematics_path = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': 
                                     kinematics.get('robot_description_kinematics', {})}
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'gui': gazebo_gui,
        }.items(),
    )
    
    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}],
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen',
    )
    
    # Controllers (with delay for spawn)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
    )
    
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
    )
    
    # MoveIt
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_param,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )
    
    # Delayed controller spawning
    delayed_controllers = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster, arm_controller],
    )
    
    return LaunchDescription(declare_args + [
        gazebo,
        rsp,
        spawn_robot,
        delayed_controllers,
        move_group,
    ])
```

---

## Gazebo Sim + MoveIt Launch

```python
#!/usr/bin/env python3
"""
Gazebo Sim (Ignition) + MoveIt2 integration launch.

Usage:
    ros2 launch pkg gz_moveit.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('pkg')
    
    # Set Gazebo resource path
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )
    
    # Arguments
    declare_args = [
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('robot_name', default_value='robot'),
    ]
    
    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    
    # Robot description
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
        ' use_gazebo:=true',
    ])
    
    robot_description_param = {'robot_description': robot_description}
    
    # Load configs
    srdf_path = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world]}.items(),
    )
    
    # Bridge (if needed for Gazebo Sim)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    )
    
    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': True}],
    )
    
    # Spawn via gz service
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen',
    )
    
    # MoveIt
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_param,
            robot_description_semantic,
            {'use_sim_time': True},
        ],
    )
    
    return LaunchDescription([
        set_env,
        *declare_args,
        gazebo,
        bridge,
        rsp,
        spawn_robot,
        move_group,
    ])
```

---

## Real Hardware Launch

```python
#!/usr/bin/env python3
"""
Real hardware + MoveIt2 launch.

Usage:
    ros2 launch pkg hardware_moveit.launch.py
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pkg')
    
    # Robot description (no Gazebo)
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
        ' use_gazebo:=false',
    ])
    
    robot_description_param = {'robot_description': robot_description}
    
    # SRDF
    srdf_path = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # Kinematics
    kinematics_path = os.path.join(pkg_share, 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)
    
    # Controllers
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['arm_controller'],
            'arm_controller': {
                'action_ns': 'follow_joint_trajectory',
                'type': 'FollowJointTrajectory',
                'default': True,
                'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
            },
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'trajectory_execution.execution_duration_monitoring': True,
    }
    
    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param],
    )
    
    # Hardware interface node (ros2_control)
    hardware_interface = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_share, 'config', 'ros2_controllers.yaml'),
            robot_description_param,
        ],
        output='screen',
    )
    
    # Controller spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )
    
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description_param,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics.get('robot_description_kinematics', {})},
            moveit_controllers,
            trajectory_execution,
        ],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        hardware_interface,
        joint_state_broadcaster,
        arm_controller,
        move_group,
    ])
```

---

## Testing/Debugging Launch

```python
#!/usr/bin/env python3
"""
Debug launch with verbose output and inspection tools.

Usage:
    ros2 launch pkg debug.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pkg')
    
    # Robot description
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
    ])
    
    robot_description_param = {'robot_description': robot_description}
    
    # SRDF
    srdf_path = os.path.join(pkg_share, 'config', 'robot.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}
    
    # TF tree publisher for debugging
    tf2_echo = Node(
        package='tf2_ros',
        executable='tf2_echo',
        arguments=['world', 'tool0'],
        output='screen',
    )
    
    # Robot state publisher with high rate
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description_param,
            {'publish_frequency': 50.0},
        ],
    )
    
    # Joint state publisher GUI
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )
    
    # MoveGroup with debug output
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[
            robot_description_param,
            robot_description_semantic,
        ],
    )
    
    return LaunchDescription([
        rsp,
        jsp,
        move_group,
        # tf2_echo,  # Uncomment to debug transforms
    ])
```

---

## Parameter Loading Patterns

### Pattern 1: Load all YAMLs in launch

```python
# Load all config files
configs = {}
config_dir = os.path.join(pkg_share, 'config')
for filename in ['kinematics.yaml', 'joint_limits.yaml', 'ompl_planning.yaml']:
    filepath = os.path.join(config_dir, filename)
    if os.path.exists(filepath):
        with open(filepath, 'r') as f:
            configs.update(yaml.safe_load(f))
```

### Pattern 2: Namespace parameters correctly

```python
# Each config needs proper namespace
params = [
    {'robot_description': robot_description},
    {'robot_description_semantic': srdf},
    {'robot_description_kinematics': kinematics},
    {'robot_description_planning': joint_limits},
    ompl_config,  # Already has 'move_group' namespace
]
```
