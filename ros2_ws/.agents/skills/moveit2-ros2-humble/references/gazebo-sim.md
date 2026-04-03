# Gazebo Sim (Ignition) Integration

## Table of Contents
1. [Setup and Launch](#setup-and-launch)
2. [Sensor Plugins](#sensor-plugins)
3. [ROS2 Bridge Configuration](#ros2-bridge-configuration)
4. [RealSense Simulation](#realsense-simulation)

## Setup and Launch

### World File Template

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_robot_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics">
    </plugin>
    <plugin
      filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="ignition-gazebo-contact-system"
      name="ignition::gazebo::systems::Contact">
    </plugin>

    <!-- Light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Work table -->
    <include>
      <uri>model://table</uri>
      <pose>0.5 0 0.4 0 0 0</pose>
      <name>work_table</name>
    </include>
  </world>
</sdf>
```

### ROS2 Control Plugin in URDF

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:macro name="ros2_control" params="name plugin">
    <ros2_control name="${name}" type="system">
      <hardware>
        <plugin>${plugin}</plugin>
      </hardware>
      
      <joint name="shoulder_pan_joint">
        <command_interface name="position">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      
      <joint name="shoulder_lift_joint">
        <command_interface name="position">
          <param name="min">-3.14</param>
          <param name="max">3.14</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
      
      <!-- Add remaining joints -->
      
    </ros2_control>
  </xacro:macro>
  
</robot>
```

### Launch File with Gazebo Sim

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('my_robot_moveit')
    
    # Set Gazebo resource paths
    set_gazebo_resources = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_share, 'worlds'])
    )
    
    gazebo_models_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([pkg_share, 'models'])
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gazebo_gui = LaunchConfiguration('gazebo_gui', default='true')
    
    # Robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments=[
            ('gz_args', PathJoinSubstitution([
                pkg_share, 'worlds', 'my_world.sdf'
            ])),
            ('on_exit_shutdown', 'false'),
        ]
    )
    
    # Spawn entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.715',
        ],
        output='screen'
    )
    
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    
    # Controllers
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller',
             '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        set_gazebo_resources,
        gazebo_models_path,
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        clock_bridge,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[arm_controller]
            )
        ),
    ])
```

## Sensor Plugins

### Camera Plugin

```xml
<!-- Add to robot URDF -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </sensor>
</gazebo>
```

### Depth Camera Plugin

```xml
<gazebo reference="depth_camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## ROS2 Bridge Configuration

### Bridge YAML Configuration

```yaml
# config/gazebo_bridge.yaml
- topic: /clock
  type: rosgraph_msgs/msg/Clock
  source: ignition
- topic: /camera/image
  type: sensor_msgs/msg/Image
  source: ignition
- topic: /camera/camera_info
  type: sensor_msgs/msg/CameraInfo
  source: ignition
- topic: /camera/depth
  type: sensor_msgs/msg/Image
  source: ignition
- topic: /camera/points
  type: sensor_msgs/msg/PointCloud2
  source: ignition
```

### Bridge Launch

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                '/camera/depth@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            ],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

## RealSense Simulation

### Simulated RealSense URDF

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <xacro:macro name="realsense_d435" params="parent prefix *origin">
    <joint name="${prefix}_camera_joint" type="fixed">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_camera_link"/>
    </joint>
    
    <link name="${prefix}_camera_link">
      <visual>
        <geometry>
          <box size="0.09 0.025 0.025"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="0.09 0.025 0.025"/>
        </geometry>
      </collision>
    </link>
    
    <!-- RGB Camera -->
    <link name="${prefix}_camera_color_frame"/>
    <joint name="${prefix}_camera_color_joint" type="fixed">
      <parent link="${prefix}_camera_link"/>
      <child link="${prefix}_camera_color_frame"/>
      <origin xyz="0 0.0175 0" rpy="0 0 0"/>
    </joint>
    
    <!-- Depth Camera -->
    <link name="${prefix}_camera_depth_frame"/>
    <joint name="${prefix}_camera_depth_joint" type="fixed">
      <parent link="${prefix}_camera_link"/>
      <child link="${prefix}_camera_depth_frame"/>
      <origin xyz="0 -0.0175 0" rpy="0 0 0"/>
    </joint>
    
    <!-- Gazebo Sensors -->
    <gazebo reference="${prefix}_camera_color_frame">
      <sensor name="${prefix}_color_camera" type="camera">
        <camera>
          <horizontal_fov>1.21126</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>${prefix}/camera/color/image_raw</topic>
      </sensor>
    </gazebo>
    
    <gazebo reference="${prefix}_camera_depth_frame">
      <sensor name="${prefix}_depth_camera" type="depth_camera">
        <camera>
          <horizontal_fov>1.21126</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>10</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>${prefix}/camera/depth/image_rect_raw</topic>
      </sensor>
    </gazebo>
  </xacro:macro>
  
</robot>
```

### Usage in Robot Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- Include RealSense macro -->
  <xacro:include filename="$(find my_robot_description)/urdf/sensors/realsense_d435.urdf.xacro"/>
  
  <!-- Robot links... -->
  
  <!-- Attach RealSense to end effector -->
  <xacro:realsense_d435 parent="wrist_3_link" prefix="camera">
    <origin xyz="0.05 0 0.02" rpy="0 -0.785 0"/>
  </xacro:realsense_d435>
  
</robot>
```
