# Troubleshooting Guide

## Table of Contents
1. [MoveIt Configuration Issues](#moveit-configuration-issues)
2. [Controller Problems](#controller-problems)
3. [Gazebo Sim Issues](#gazebo-sim-issues)
4. [Perception/Camera Issues](#perceptioncamera-issues)
5. [TF and Frame Issues](#tf-and-frame-issues)

## MoveIt Configuration Issues

### "No planning library loaded"

**Symptoms:**
```
[move_group]: No planning library loaded
```

**Solutions:**
1. Install OMPL: `sudo apt install ros-humble-ompl`
2. Check `ompl_planning.yaml` path in launch file
3. Verify `planning_plugin` is set correctly:
   ```yaml
   planning_plugin: ompl_interface/OMPLPlanner
   request_adapters: ...
   ```

### "Group 'manipulator' not found in SRDF"

**Symptoms:**
```
[move_group]: Group 'manipulator' not found in SRDF
```

**Solutions:**
1. Verify SRDF is being loaded correctly
2. Check group name matches in SRDF:
   ```xml
   <group name="manipulator">
     <chain base_link="base_link" tip_link="tool0"/>
   </group>
   ```
3. Ensure robot_description_semantic parameter is set

### Planning fails with "Start state is invalid"

**Symptoms:**
```
[move_group.plan]: Planning request failed
Start state is invalid
```

**Solutions:**
1. Check joint limits in `joint_limits.yaml`
2. Verify robot is not in collision:
   ```bash
   ros2 service call /planning_scene_world get_planning_scene
   ```
3. Set proper initial joint values before planning:
   ```python
   manipulator.set_start_state_to_current_state()
   ```

## Controller Problems

### "Controller arm_controller not found"

**Symptoms:**
```
[ros2_control]: Controller 'arm_controller' not found
```

**Solutions:**
1. Check controller is defined in `ros2_controllers.yaml`:
   ```yaml
   controller_manager:
     ros__parameters:
       arm_controller:
         type: joint_trajectory_controller/JointTrajectoryController
   ```
2. Verify controller configuration exists:
   ```yaml
   arm_controller:
     ros__parameters:
       joints:
         - shoulder_pan_joint
         - shoulder_lift_joint
         # ... all joints
   ```
3. Load controller after spawn:
   ```bash
   ros2 control load_controller --set-state active arm_controller
   ```

### Joint trajectory not executing

**Symptoms:**
```
[arm_controller]: Can't accept new action goals. Controller is not running.
```

**Solutions:**
1. Check controller state:
   ```bash
   ros2 control list_controllers
   ```
2. Activate controller:
   ```bash
   ros2 control set_controller_state arm_controller active
   ```
3. Verify joint names match exactly between URDF and controller config

### "Command interfaces not available"

**Symptoms:**
```
[ros2_control]: Command interface 'position' not available
```

**Solutions:**
1. Check ROS2 control plugin in URDF:
   ```xml
   <ros2_control name="arm" type="system">
     <hardware>
       <plugin>gz_ros2_control/GazeboSimSystem</plugin>
     </hardware>
     <joint name="...">
       <command_interface name="position"/>
       <state_interface name="position"/>
     </joint>
   </ros2_control>
   ```
2. Ensure `joint_state_broadcaster` is loaded before other controllers

## Gazebo Sim Issues

### Robot doesn't spawn

**Symptoms:**
```
[create]: Entity not found
```

**Solutions:**
1. Check Gazebo resource paths:
   ```bash
   echo $IGN_GAZEBO_RESOURCE_PATH
   ```
2. Verify mesh paths use `package://` protocol
3. Check for xacro errors:
   ```bash
   xacro robot.urdf.xacro > robot.urdf
   ```
4. Ensure `/robot_description` topic is published before spawning

### Black screen in Gazebo

**Symptoms:** Gazebo launches but viewport is black

**Solutions:**
1. Try software rendering:
   ```bash
   export LIBGL_ALWAYS_SOFTWARE=1
   ```
2. Update graphics drivers
3. Check for conflicting environment variables:
   ```bash
   unset GTK_PATH
   unset GIO_MODULE_DIR
   ```

### Physics explosions

**Symptoms:** Robot flies away or shakes violently

**Solutions:**
1. Reduce simulation step size:
   ```xml
   <physics name="1ms" type="ignored">
     <max_step_size>0.001</max_step_size>
   </physics>
   ```
2. Check for missing inertia in URDF:
   ```xml
   <inertial>
     <mass>1.0</mass>
     <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
   </inertial>
   ```
3. Verify joint limits are reasonable

## Perception/Camera Issues

### No images from simulated camera

**Symptoms:** Topic exists but no data published

**Solutions:**
1. Check bridge configuration:
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /camera/image
   ```
2. Verify sensor topic matches bridge:
   ```xml
   <topic>camera/color/image_raw</topic>
   ```
3. Check Gazebo sensor is enabled:
   ```xml
   <always_on>1</always_on>
   <update_rate>30</update_rate>
   ```

### Point cloud not published

**Symptoms:** `/camera/points` topic empty

**Solutions:**
1. Enable point cloud in RealSense:
   ```python
   parameters=[{
       'enable_pointcloud': True,
       'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
   }]
   ```
2. Check depth image is being published first
3. For simulation, verify depth_camera plugin is configured

### TF lookup fails for camera frame

**Symptoms:**
```
[point_cloud_processor]: Could not transform...
```

**Solutions:**
1. Publish static transform:
   ```python
   static_tf = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['0.5', '0', '1', '0', '0', '0', 'base_link', 'camera_link']
   )
   ```
2. Check frame_id in camera messages matches TF tree
3. Use `ros2 run tf2_tools view_frames` to debug TF tree

## TF and Frame Issues

### "Could not find transform from X to Y"

**Symptoms:**
```
[move_group]: Could not find transform from base_link to tool0
```

**Solutions:**
1. Verify all links are connected in URDF
2. Check robot_state_publisher is running:
   ```bash
   ros2 topic echo /tf
   ```
3. Ensure `use_sim_time` is consistent across nodes
4. Check for cyclical joint definitions

### Wrong robot pose in RViz

**Symptoms:** Robot appears in wrong position/orientation

**Solutions:**
1. Check Fixed Frame in RViz is set to `base_link` or `world`
2. Verify initial joint positions in `initial_positions.yaml`
3. Check spawn position in Gazebo launch:
   ```python
   arguments=['-x', '0.25', '-y', '0', '-z', '0.715']
   ```

### Planning in wrong reference frame

**Symptoms:** Motion plans to wrong location

**Solutions:**
1. Set correct frame_id in PoseStamped:
   ```python
   pose.header.frame_id = "base_link"  # or "world"
   ```
2. Verify virtual joint in SRDF:
   ```xml
   <virtual_joint name="fixed_base" type="fixed" 
                  parent_frame="world" child_link="base_link"/>
   ```
3. Check planning scene frame matches pose frame

## Common Error Messages

### "Failed to load robot model"

```bash
# Check URDF is valid
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro robot.urdf.xacro)"

# Verify all package paths
ros2 pkg prefix my_robot_description
```

### "No kinematics solver instantiated"

```bash
# Install KDL plugin
sudo apt install ros-humble-moveit-resources-prbt-moveit-config

# Check kinematics.yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

### "Trajectory execution failed"

```bash
# Check trajectory is within limits
ros2 topic echo /arm_controller/joint_trajectory

# Verify controller is active
ros2 control list_controllers
```
