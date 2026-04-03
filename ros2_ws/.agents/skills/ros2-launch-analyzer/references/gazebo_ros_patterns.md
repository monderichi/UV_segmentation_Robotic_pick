# Gazebo ROS Patterns

Common patterns for integrating Gazebo simulation with ROS2.

## Table of Contents

1. [Basic Gazebo Launch](#basic-gazebo-launch)
2. [Robot Spawning](#robot-spawning)
3. [Controller Management](#controller-management)
4. [Robot Description for Gazebo](#robot-description-for-gazebo)
5. [World Files](#world-files)
6. [Common Integration Patterns](#common-integration-patterns)

---

## Basic Gazebo Launch

### Gazebo Server and Client

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

# Gazebo server (physics simulation)
gzserver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
    ]),
    launch_arguments={"world": world_file}.items()
)

# Gazebo client (GUI)
gzclient = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
    ]),
    condition=IfCondition(gazebo_gui),  # Optional conditional
)
```

### World File Path

```python
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

world_file = PathJoinSubstitution([
    FindPackageShare("package_name"), "worlds", "environment.world"
])
```

Or as string for OpaqueFunction:

```python
import os
from ament_index_python.packages import get_package_share_directory

world_file = os.path.join(
    get_package_share_directory('package_name'),
    'worlds',
    'table_world.world'
)
```

---

## Robot Spawning

### Spawn Entity Node

```python
from launch_ros.actions import Node

spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
        "-entity", "robot_name",           # Entity name in Gazebo
        "-topic", "robot_description",     # Topic with URDF
        "-x", "0.0",                        # X position
        "-y", "0.0",                        # Y position
        "-z", "0.0",                        # Z position
        "-R", "0.0",                        # Roll
        "-P", "0.0",                        # Pitch
        "-Y", "0.0",                        # Yaw
    ],
    output="screen",
)
```

### Spawn from File

```python
spawn_robot = Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
        "-entity", "robot_name",
        "-file", "/path/to/robot.urdf",    # Direct file path
        "-x", "0.25",
        "-y", "0",
        "-z", "0.715",
    ],
    output="screen",
)
```

---

## Controller Management

### Joint State Broadcaster

```python
joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
)
```

### Joint Trajectory Controller

```python
initial_joint_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
)
```

### Stopped Controller

```python
from launch.conditions import UnlessCondition

stopped_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--stopped"],
    condition=UnlessCondition(start_joint_controller),
)
```

### Controller Configuration YAML

Example `ur_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    forward_command_controller_position:
      type: forward_command_controller/ForwardCommandController

joint_trajectory_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 100.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.2
      goal_time: 0.0
      shoulder_pan_joint:
        trajectory: 0.2
        goal: 0.0
      # ... other joints
```

---

## Robot Description for Gazebo

### XACRO with Gazebo Parameters

```python
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

robot_description_content = Command([
    PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
    PathJoinSubstitution([FindPackageShare("package_name"), "urdf", "robot.urdf.xacro"]),
    " ",
    "safety_limits:=", safety_limits, " ",
    "safety_pos_margin:=", safety_pos_margin, " ",
    "safety_k_position:=", safety_k_position, " ",
    "name:=ur ur_type:=", ur_type, " ",
    "prefix:=", prefix, " ",
    "sim_gazebo:=true ",                    # Enable Gazebo simulation
    "simulation_controllers:=", initial_joint_controllers, " ",
    "initial_positions_file:=", initial_positions_file_abs, " ",
    # Essential UR parameters
    "joint_limit_params:=", PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"
    ]), " ",
    "kinematics_params:=", PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"
    ]), " ",
    "physical_params:=", PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"
    ]), " ",
    "visual_params:=", PathJoinSubstitution([
        FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"
    ])
])

robot_description = {"robot_description": robot_description_content}
```

### Robot State Publisher

```python
robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[{"use_sim_time": True}, robot_description],
)
```

---

## World Files

### Basic World Structure

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Custom object -->
    <model name="table">
      <include>
        <uri>model://table</uri>
      </include>
      <pose>0.5 0 0 0 0 0</pose>
    </model>
  </world>
</sdf>
```

### World with Plugin

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="custom_world">
    <physics type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    
    <plugin name="gazebo_ros_clock" filename="libgazebo_ros_clock.so"/>
    
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
  </world>
</sdf>
```

---

## Common Integration Patterns

### UR Robot + Gazebo + MoveIt2

Complete pattern for UR robot simulation:

```python
def launch_setup(context, *args, **kwargs):
    # ... get launch configurations ...
    
    # 1. Build robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " sim_gazebo:=true",
        " simulation_controllers:=", initial_joint_controllers,
        # ... other xacro args
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    # 3. Controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    initial_joint_controller_start = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
    )
    
    # 4. Gazebo
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"])
        ]),
        launch_arguments={"world": world_file}.items()
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"])
        ]),
        condition=IfCondition(gazebo_gui),
    )
    
    # 5. Spawn Robot
    spawn_ur = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "ur",
            "-topic", "robot_description",
            "-x", "0.25",
            "-y", "0",
            "-z", "0.715"
        ],
        output="screen",
    )
    
    # 6. MoveIt (optional, via include)
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
        ]),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "use_fake_hardware": "true",
        }.items(),
    )
    
    return [
        robot_state_publisher_node,
        joint_state_broadcaster,
        initial_joint_controller_start,
        gzserver,
        gzclient,
        spawn_ur,
        ur_moveit_launch,
    ]
```

### Delayed Node Launch

Launch a node after simulation stabilizes:

```python
from launch.actions import TimerAction, ExecuteProcess

delayed_script = TimerAction(
    period=10.0,  # 10 second delay
    actions=[
        ExecuteProcess(
            cmd=["ros2", "run", "package_name", "script.py"],
            shell=True,
            output="screen"
        )
    ]
)
```

### Timer with Node

```python
delayed_node = TimerAction(
    period=5.0,
    actions=[
        Node(
            package="package_name",
            executable="node_executable",
            parameters=[{"use_sim_time": True}],
        )
    ]
)
```

---

## Environment Variables

### Gazebo Plugin Path

```bash
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/custom/models
```

### In Launch Script

```python
from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable(
    name="GAZEBO_MODEL_PATH",
    value="/path/to/models"
),
```

Or in shell wrapper:

```python
import os
# In the launch file setup
os.environ['GAZEBO_PLUGIN_PATH'] = '/opt/ros/humble/lib'
```
