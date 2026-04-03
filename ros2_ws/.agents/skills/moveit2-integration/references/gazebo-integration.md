# Gazebo Integration with MoveIt2

Complete guide for integrating MoveIt2 with Gazebo simulation environments.

## Table of Contents

1. [Gazebo Classic vs Gazebo Sim](#gazebo-classic-vs-gazebo-sim)
2. [Package Dependencies](#package-dependencies)
3. [URDF Configuration](#urdf-configuration)
4. [ros2_control Setup](#ros2_control-setup)
5. [Controller Configuration](#controller-configuration)
6. [Launch File Patterns](#launch-file-patterns)
7. [Common Issues](#common-issues)

---

## Gazebo Classic vs Gazebo Sim

| Feature | Gazebo Classic | Gazebo Sim (Ignition) |
|---------|----------------|----------------------|
| Package | `gazebo_ros` | `ros_gz_sim` |
| Plugin lib | `libgazebo_ros2_control.so` | `libgz_ros2_control-system.so` |
| Launch file | `gazebo.launch.py` | `gz_sim.launch.py` |
| Resource path | `GAZEBO_MODEL_PATH` | `GZ_SIM_RESOURCE_PATH` |
| Spawn | `spawn_entity.py` | `ros_gz_sim create` |
| Bridge | Not needed | `ros_gz_bridge` |

### Key Differences

**Gazebo Classic (11):**
```xml
<gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find pkg)/config/controllers.yaml</parameters>
    </plugin>
</gazebo>
```

**Gazebo Sim (Harmonic/Ionic):**
```xml
<gazebo reference="base_link">
    <plugin filename="libgz_ros2_control-system.so" 
            name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <parameters>$(find pkg)/config/controllers.yaml</parameters>
    </plugin>
</gazebo>
```

**Cannot mix both in same simulation!**

---

## Package Dependencies

### Gazebo Classic

```xml
<!-- package.xml -->
<depend>gazebo_ros</depend>
<depend>gazebo_ros2_control</depend>
<depend>gazebo_plugins</depend>

<!-- For specific ROS2 distro -->
<depend>ros_humble_gazebo_ros</depend>
<depend>ros_humble_gazebo_ros2_control</depend>
```

### Gazebo Sim

```xml
<!-- package.xml -->
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
<depend>gz_ros2_control</depend>

<!-- Or specific sub-packages -->
<depend>ros_gz_sim</depend>
<depend>ros_gz_bridge</depend>
```

### Install Commands

```bash
# Gazebo Classic (Humble)
sudo apt install ros-humble-gazebo-ros \
                 ros-humble-gazebo-ros2-control \
                 ros-humble-gazebo-plugins

# Gazebo Sim (Humble)
sudo apt install ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge \
                 ros-humble-gz-ros2-control

# Jazzy (Gazebo Sim is default)
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-gz-ros2-control
```

---

## URDF Configuration

### Conditional Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
    
    <xacro:arg name="use_gazebo" default="false"/>
    <xacro:arg name="gazebo_version" default="classic"/>  <!-- or "sim" -->
    
    <!-- Robot description -->
    <xacro:include filename="arm.xacro"/>
    <xacro:arm/>
    
    <!-- Conditional ros2_control -->
    <xacro:if value="$(arg use_gazebo)">
        <xacro:if value="${gazebo_version == 'classic'}">
            <xacro:include filename="gazebo_classic_control.xacro"/>
            <xacro:gazebo_classic_control/>
        </xacro:if>
        <xacro:if value="${gazebo_version == 'sim'}">
            <xacro:include filename="gazebo_sim_control.xacro"/>
            <xacro:gazebo_sim_control/>
        </xacro:if>
    </xacro:if>
    <xacro:unless value="$(arg use_gazebo)">
        <xacro:include filename="hardware_control.xacro"/>
        <xacro:hardware_control/>
    </xacro:unless>
    
</robot>
```

### Gazebo Classic Control Xacro

```xml
<!-- gazebo_classic_control.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

<xacro:macro name="gazebo_classic_control">
    
    <!-- Gazebo plugin -->
    <gazebo>
        <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
            <parameters>$(find pkg)/config/ros2_controllers.yaml</parameters>
        </plugin>
    </gazebo>
    
    <!-- ros2_control hardware interface -->
    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        
        <joint name="joint1">
            <command_interface name="position"/>
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
        
        <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        
        <!-- ... more joints -->
    </ros2_control>
    
</xacro:macro>

</robot>
```

### Gazebo Sim Control Xacro

```xml
<!-- gazebo_sim_control.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

<xacro:macro name="gazebo_sim_control">
    
    <!-- Gazebo Sim plugin (attached to base link) -->
    <gazebo reference="base_link">
        <plugin filename="libgz_ros2_control-system.so" 
                name="gz_ros2_control::GazeboSimROS2ControlPlugin">
            <parameters>$(find pkg)/config/ros2_controllers.yaml</parameters>
        </plugin>
    </gazebo>
    
    <!-- ros2_control hardware interface -->
    <ros2_control name="GazeboSimSystem" type="system">
        <hardware>
            <plugin>gz_ros2_control/GazeboSimSystem</plugin>
        </hardware>
        
        <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        
        <!-- ... more joints -->
    </ros2_control>
    
</xacro:macro>

</robot>
```

### Material Properties for Gazebo

```xml
<!-- Add to each link for proper visual/material in Gazebo -->
<gazebo reference="link1">
    <material>Gazebo/Orange</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
</gazebo>
```

---

## ros2_control Setup

### Controller Manager Configuration

```yaml
# config/ros2_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    # Broadcasters
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    
    # Controllers
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    
    gripper_controller:
      type: position_controllers/GripperActionController
```

### Joint Trajectory Controller

```yaml
arm_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    
    command_interfaces:
      - position
    
    state_interfaces:
      - position
      - velocity
    
    # Smoothing
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    
    # Allow partial goals (not all joints specified)
    allow_partial_joints_goal: false
    
    # Constraints
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.5
      
      joint1:
        trajectory: 0.1
        goal: 0.05
      joint2:
        trajectory: 0.1
        goal: 0.05
      # ...
```

### Gripper Controller

```yaml
gripper_controller:
  ros__parameters:
    joint: gripper_joint
    
    constraints:
      goal: 0.01
      trajectory: 0.1
```

### Forward Command Controller (for velocity/effort control)

```yaml
velocity_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
    interface_name: velocity
```

---

## Controller Configuration

### MoveIt Controllers

```yaml
# config/moveit_controllers.yaml
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller
  
  arm_controller:
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
  
  gripper_controller:
    action_ns: gripper_cmd
    type: GripperCommand
    default: true
    joints:
      - gripper_joint
      - gripper_joint2

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
```

### Trajectory Execution

```python
# In launch file
trajectory_execution = {
    'moveit_manage_controllers': False,
    'trajectory_execution.allowed_execution_duration_scaling': 1.2,
    'trajectory_execution.allowed_goal_duration_margin': 0.5,
    'trajectory_execution.allowed_start_tolerance': 0.01,
    'trajectory_execution.execution_duration_monitoring': False,
}
```

---

## Launch File Patterns

### Gazebo Classic Launch

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDeclaration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'world': 'empty.world'}.items(),
    )
    
    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )
    
    # Spawn
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
    )
    
    # Controllers
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
    
    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        joint_state_broadcaster,
        arm_controller,
    ])
```

### Gazebo Sim Launch

```python
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('pkg')
    
    # Set resource path
    set_env = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, 'models')
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r -v4 empty.sdf'}.items(),
    )
    
    # Bridge (optional - for specific message types)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )
    
    # Spawn via ros_gz_sim create
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robot'],
    )
    
    return LaunchDescription([
        set_env,
        gazebo,
        bridge,
        spawn,
    ])
```

---

## Common Issues

### Gazebo Classic Issues

**Issue:** `libgazebo_ros2_control.so` not found
```bash
# Install missing package
sudo apt install ros-$ROS_DISTRO-gazebo-ros2-control

# Verify library exists
find /opt/ros -name "libgazebo_ros2_control.so"
```

**Issue:** Robot spawns but controllers fail
```bash
# Check plugin loaded in Gazebo GUI (Model → Plugins)
# Verify xacro processed correctly
xacro robot.urdf.xacro use_gazebo:=true > /tmp/robot.urdf
grep -A 5 gazebo_ros2_control /tmp/robot.urdf
```

### Gazebo Sim Issues

**Issue:** Resource not found
```bash
# Set correct environment variable
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/models

# Check resource path
gz sim --list-resources
```

**Issue:** Plugin not loading
```bash
# Verify plugin name in URDF
# Gazebo Sim uses gz_ros2_control::GazeboSimROS2ControlPlugin
# Not gazebo_ros2_control!
```

### Controller Issues

**Issue:** Controllers not starting
```bash
# Check update_rate is reasonable (not too high)
# Verify joint names match exactly
ros2 control list_hardware_interfaces
```

**Issue:** High latency in Gazebo
```yaml
# Reduce update_rate in controller_manager
controller_manager:
  ros__parameters:
    update_rate: 50  # Instead of 1000
```

---

## Testing Commands

```bash
# Check Gazebo is publishing joint states
ros2 topic echo /joint_states

# Send test command
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    '{joint_names: [joint1], points: [{positions: [0.5], time_from_start: {sec: 2}}]}'

# Check controller state
ros2 control list_controllers -v

# Verify action server
ros2 action info /arm_controller/follow_joint_trajectory
```
