---
name: moveit2-ros2-humble
description: Complete MoveIt2 integration for ROS2 Humble with robotic arms and Intel RealSense cameras. Use when setting up MoveIt2 motion planning, configuring robot descriptions and SRDF files, integrating RGB-D cameras for perception, creating motion planners, or building perception-based grasping pipelines. Covers Gazebo Sim (Ignition) simulation, ROS2 controllers, point cloud processing, and visual servoing.
---

# MoveIt2 ROS2 Humble Skill

Complete guide for integrating robotic arms with MoveIt2 on ROS2 Humble, including Intel RealSense camera integration for perception-based manipulation.

## Quick Start

### 1. Robot Description Package Structure

```
my_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robots/
│   │   └── my_robot.urdf.xacro
│   ├── control/
│   │   └── ros2_control.urdf.xacro
│   └── sensors/
│       └── realsense_d435.urdf.xacro
└── meshes/
    └── collision/
```

### 2. Basic Robot Xacro Template

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  
  <!-- Import macros -->
  <xacro:include filename="$(find my_robot_description)/urdf/control/ros2_control.urdf.xacro"/>
  
  <!-- Base link -->
  <link name="base_link"/>
  
  <!-- Robot arm links and joints -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <mesh filename="package://my_robot_description/meshes/shoulder_link.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.2"/>
      </geometry>
    </collision>
  </link>
  
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="3.14"/>
  </joint>
  
  <!-- Add remaining joints: shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 -->
  
  <!-- Tool frame -->
  <link name="tool0"/>
  <joint name="tool0_joint" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="tool0"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>
  
  <!-- ROS2 Control -->
  <xacro:ros2_control name="my_robot_arm" plugin="gz_ros2_control/GazeboSimSystem"/>
  
</robot>
```

### 3. SRDF (Semantic Robot Description)

Create `config/my_robot.srdf`:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <group_state name="home" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.57"/>
    <joint name="elbow_joint" value="1.57"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base_link"/>
  
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <!-- Add remaining disable_collisions -->
</robot>
```

### 4. Launch File

```python
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, 
    ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    pkg_share = FindPackageShare('my_robot_moveit')
    
    # Launch args
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot state publisher
    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
            ]),
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
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-entity', 'my_robot'],
        output='screen'
    )
    
    # Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    # MoveIt
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'kinematics.yaml']),
            PathJoinSubstitution([pkg_share, 'config', 'ompl_planning.yaml']),
            {'robot_description_semantic': Command([
                'cat ', PathJoinSubstitution([pkg_share, 'config', 'my_robot.srdf'])
            ])},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        robot_description,
        gazebo,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_broadcaster])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=load_joint_state_broadcaster, on_exit=[load_arm_controller])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=load_arm_controller, on_exit=[move_group])
        )
    ])
```

## Intel RealSense Integration

### Camera URDF

```xml
<!-- Include in robot xacro -->
<link name="camera_link">
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

<joint name="camera_joint" type="fixed">
  <parent link="wrist_3_link"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.02" rpy="0 -0.785 0"/>
</joint>

<!-- Camera optical frame (Z forward for OpenCV) -->
<link name="camera_color_optical_frame"/>
<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_color_optical_frame"/>
  <origin rpy="-1.5708 0 -1.5708" xyz="0 0 0"/>
</joint>
```

### Point Cloud Processing Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseArray, Pose
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix, quaternion_from_matrix

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('point_cloud_processor')
        
        self.subscription = self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points', 
            self.cloud_callback, 10
        )
        self.pose_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
    def cloud_callback(self, msg: PointCloud2):
        # Convert to numpy array
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        
        if len(points) == 0:
            return
        
        # Simple clustering (replace with actual object detection)
        # This is a placeholder - use proper segmentation
        detected_poses = PoseArray()
        detected_poses.header = msg.header
        
        # Example: Find centroid of points above table
        height_threshold = 0.1  # 10cm above table
        objects = points[points[:, 2] > height_threshold]
        
        if len(objects) > 0:
            centroid = np.mean(objects, axis=0)
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.w = 1.0
            detected_poses.poses.append(pose)
        
        self.pose_pub.publish(detected_poses)

def main():
    rclpy.init()
    node = PointCloudProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## MoveIt2 Configuration Files

### kinematics.yaml

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
```

### ros2_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
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
    gains:
      shoulder_pan_joint: {p: 100.0, i: 0.0, d: 10.0}
      shoulder_lift_joint: {p: 100.0, i: 0.0, d: 10.0}
      elbow_joint: {p: 100.0, i: 0.0, d: 10.0}
      wrist_1_joint: {p: 50.0, i: 0.0, d: 5.0}
      wrist_2_joint: {p: 50.0, i: 0.0, d: 5.0}
      wrist_3_joint: {p: 50.0, i: 0.0, d: 5.0}
```

### joint_limits.yaml

```yaml
joint_limits:
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  shoulder_lift_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  elbow_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  wrist_1_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
  wrist_2_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
  wrist_3_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
```

## Motion Planning with Python

### Basic Motion Planning

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.kinematic_constraints import construct_joint_constraint
from geometry_msgs.msg import PoseStamped

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.moveit = MoveItPy(node_name="moveit_py")
        self.manipulator = self.moveit.get_planning_component("manipulator")
    
    def plan_to_joint_goal(self, joint_values):
        """Plan to specific joint configuration"""
        self.manipulator.set_start_state_to_current_state()
        
        constraints = {
            "joint_constraints": [
                {"joint_name": name, "position": value}
                for name, value in zip(
                    ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
                    joint_values
                )
            ]
        }
        self.manipulator.set_goal_state(constraints=constraints)
        
        plan_result = self.manipulator.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])
    
    def plan_to_pose_goal(self, x, y, z, roll, pitch, yaw):
        """Plan to Cartesian pose"""
        self.manipulator.set_start_state_to_current_state()
        
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Convert roll, pitch, yaw to quaternion
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        self.manipulator.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")
        
        plan_result = self.manipulator.plan()
        if plan_result:
            self.moveit.execute(plan_result.trajectory, controllers=[])

def main():
    rclpy.init()
    planner = MotionPlanner()
    # Example: go home
    planner.plan_to_joint_goal([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Cartesian Path Planning

```python
def plan_cartesian_path(self, waypoints):
    """
    Plan Cartesian path through waypoints
    waypoints: list of Pose
    """
    from moveit.core.kinematic_constraints import construct_joint_constraint
    
    self.manipulator.set_start_state_to_current_state()
    
    # Set Cartesian path constraints
    self.manipulator.set_goal_state(waypoints=waypoints, 
                                     planning_group="manipulator",
                                     link_name="tool0")
    
    # Configure planning parameters
    plan_params = {
        "planning_attempts": 10,
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "eef_step": 0.01,  # 1cm steps
        "jump_threshold": 0.0  # Disable jump threshold checking
    }
    
    plan_result = self.manipulator.plan(plan_params)
    return plan_result
```

## Visual Servoing

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from moveit.planning import MoveItPy

class VisualServoing(Node):
    """IBVS (Image-Based Visual Servoing) for grasping"""
    
    def __init__(self):
        super().__init__('visual_servoing')
        
        self.moveit = MoveItPy(node_name="moveit_py_vs")
        self.manipulator = self.moveit.get_planning_component("manipulator")
        
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.image_callback, 10
        )
        self.vel_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10
        )
        
        self.bridge = CvBridge()
        self.target_center = (320, 240)  # Image center for 640x480
        self.Kp = 0.001  # Proportional gain
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Detect object (simple color-based detection)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Example: detect red objects
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find centroid
        M = cv2.moments(mask)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Compute error
            error_x = self.target_center[0] - cx
            error_y = self.target_center[1] - cy
            
            # Generate velocity command
            vel_msg = TwistStamped()
            vel_msg.header = msg.header
            vel_msg.header.frame_id = "tool0"
            
            # Cartesian velocities to center object
            vel_msg.twist.linear.y = self.Kp * error_x  # Left/right
            vel_msg.twist.linear.z = -self.Kp * error_y  # Up/down
            
            self.vel_pub.publish(vel_msg)

def main():
    rclpy.init()
    node = VisualServoing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Reference Documentation

- **Perception Pipelines**: See [references/perception.md](references/perception.md) for point cloud processing, object detection integration, and depth image handling
- **Gazebo Sim Integration**: See [references/gazebo-sim.md](references/gazebo-sim.md) for simulation-specific setup, sensor plugins, and bridging
- **Common Issues**: See [references/troubleshooting.md](references/troubleshooting.md) for solutions to frequent problems
- **SRDF Examples**: See [references/srdf-examples.md](references/srdf-examples.md) for complex SRDF configurations including end-effectors and collision matrices

## Best Practices

1. **Joint Naming**: Use UR-standard names (`shoulder_pan_joint`, `elbow_joint`, etc.) for maximum compatibility
2. **Collision Meshes**: Use simplified convex hulls for collision, detailed meshes for visual
3. **Controller Rates**: Use 100Hz minimum for trajectory controllers in simulation
4. **TF Tree**: Ensure all frames are connected through the TF tree
5. **Planning Scene**: Update planning scene with detected objects before planning
6. **Servo Tuning**: Start with low gains (0.001-0.01) and increase gradually
