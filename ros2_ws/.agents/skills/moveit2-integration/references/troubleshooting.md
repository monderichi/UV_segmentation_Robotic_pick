# MoveIt2 Troubleshooting Guide

Common issues and solutions for MoveIt2 integration.

## Table of Contents

- [Planning Failures](#planning-failures)
- [Execution Failures](#execution-failures)
- [Controller Issues](#controller-issues)
- [TF/Transform Issues](#tftransform-issues)
- [Performance Issues](#performance-issues)

---

## Planning Failures

### "No IK solution found"

**Symptoms:** Planning fails immediately with IK-related errors.

**Causes & Solutions:**

1. **Goal out of reach**
   - Check goal pose is within workspace
   - Verify using FK: `ros2 run tf2_ros tf2_echo base_link tool0`

2. **IK solver configuration issue**
   - Verify `kinematics.yaml` is loaded correctly
   - Check solver plugin name is correct
   - Increase `kinematics_solver_timeout`

3. **Joint limits too restrictive**
   - Check `joint_limits.yaml` for overly strict limits
   - Verify URDF joint limits match

```bash
# Debug IK
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'arm',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {position: {x: 0.5, y: 0, z: 0.3}, orientation: {w: 1}}
    }
  }
}"
```

### "Start state in collision"

**Symptoms:** Planning fails because starting configuration is in collision.

**Solutions:**

1. **Check current robot state:**
   ```bash
   ros2 topic echo /joint_states
   ```

2. **Visualize in RViz:** Check if robot model overlaps with obstacles

3. **Use FixStartStateCollision adapter:**
   ```yaml
   # ompl_planning.yaml
   request_adapters: >
     default_planner_request_adapters/FixStartStateCollision
   ```

4. **Move robot to valid state manually:**
   ```python
   # Use a named state that's known to be valid
   arm.set_goal_state(configuration_name="home")
   ```

### "Goal state in collision"

**Symptoms:** Goal configuration would result in collision.

**Solutions:**

1. **Check collision objects in scene**
2. **Verify goal pose doesn't intersect obstacles**
3. **Adjust goal tolerance if near collision:**
   ```python
   arm.set_goal_position_tolerance(0.01)
   arm.set_goal_orientation_tolerance(0.1)
   ```

---

## Execution Failures

### "Controller failed during execution"

**Symptoms:** Trajectory sent but robot doesn't move or stops abruptly.

**Causes & Solutions:**

1. **Trajectory violates joint limits**
   - Check `joint_limits.yaml` acceleration limits
   - Verify velocity limits match hardware capabilities

2. **Path tolerance exceeded**
   - Joint trajectory controller aborts if error is too large
   - Increase `path_tolerance` in controller config:
   ```yaml
   joint_trajectory_controller:
     ros__parameters:
       constraints:
         goal_time: 0.0
         stopped_velocity_tolerance: 0.0
         joint1:
           trajectory: 0.05
           goal: 0.03
   ```

3. **Goal tolerance not met**
   - Final position is outside tolerance window
   - Increase goal tolerance or check calibration

### "Trajectory message contains waypoints that are not strictly increasing in time"

**Causes & Solutions:**

1. **Time parameterization missing**
   - Ensure `AddTimeOptimalParameterization` adapter is in OMPL config
   - Or manually parameterize:
   ```python
   from moveit.core.robot_trajectory import RobotTrajectory
   trajectory = RobotTrajectory(robot_model, "arm")
   trajectory.set_robot_trajectory_msg(current_state, trajectory_msg)
   trajectory.apply_totg_time_parameterization(velocity_scaling=0.1)
   ```

---

## Controller Issues

### "Action client not connected"

**Symptoms:** MoveIt2 cannot connect to trajectory controller.

**Debug Steps:**

1. **Check controller is loaded:**
   ```bash
   ros2 control list_controllers
   ```
   Should show `joint_trajectory_controller` in `active` state.

2. **Verify action server exists:**
   ```bash
   ros2 action list | grep follow_joint_trajectory
   ```

3. **Check namespace matches:**
   - Controller: `/joint_trajectory_controller/follow_joint_trajectory`
   - MoveIt config must match in `moveit_controllers.yaml`

4. **Verify joints match:**
   - Controller joints must match planning group joints
   - Check case sensitivity

### "Controller is taking too long to execute trajectory"

**Solutions:**

1. **Check simulation time:**
   ```bash
   ros2 param get /gazebo use_sim_time
   ros2 param get /move_group use_sim_time
   ```
   Must match - if Gazebo uses sim time, all nodes should.

2. **Verify clock is publishing:**
   ```bash
   ros2 topic hz /clock  # Should show ~1000Hz in Gazebo
   ```

---

## TF/Transform Issues

### "Failed to fetch current robot state"

**Symptoms:** MoveIt2 cannot determine current joint positions.

**Solutions:**

1. **Check joint_states topic:**
   ```bash
   ros2 topic echo /joint_states
   ```

2. **Verify joint_state_broadcaster is active:**
   ```bash
   ros2 control list_controllers
   ```

3. **Check robot_description:**
   ```bash
   ros2 param get /robot_state_publisher robot_description
   ```

4. **Verify all joint names match** between URDF and controller config

### "Lookup would require extrapolation into the future"

**Causes:** TF buffer doesn't have transform for the requested time.

**Solutions:**

1. **Increase TF buffer timeout:**
   ```python
   moveit = MoveItPy(
       node_name="moveit_py",
       tf_buffer_duration=30.0
   )
   ```

2. **Check for clock desynchronization** between nodes

3. **Verify all nodes use same time source** (sim or wall)

---

## Performance Issues

### Planning is too slow

**Solutions:**

1. **Use faster planner:**
   ```yaml
   arm:
     planner_configs:
       - RRTConnectkConfigDefault  # Fastest general purpose
   ```

2. **Reduce planning attempts:**
   ```yaml
   arm:
     planning_attempts: 1  # Default is often higher
   ```

3. **Reduce planning time:**
   ```python
   arm.set_planning_time(1.0)  # seconds
   ```

4. **Simplify collision checking:**
   ```yaml
   # ompl_planning.yaml
   longest_valid_segment_fraction: 0.01  # Increase for faster checking
   ```

### High CPU usage

**Solutions:**

1. **Reduce collision checking resolution**
2. **Limit planning scene updates:**
   ```python
   planning_scene_monitor = moveit.get_planning_scene_monitor()
   planning_scene_monitor.start_scene_monitor()
   # Don't update scene on every change
   ```

3. **Use simplified collision models** in URDF

---

## Debug Tools

### Enable Verbose Logging

```bash
ros2 run moveit_ros_move_group move_group --ros-args --log-level debug
```

### Visualize Planning Scene

```bash
ros2 launch moveit_ros_visualization moveit_rviz.launch.py
```

### Test IK Service Directly

```python
#!/usr/bin/env python3
import rclpy
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node("ik_test")
client = node.create_client(GetPositionIK, "/compute_ik")

request = GetPositionIK.Request()
request.ik_request.group_name = "arm"
request.ik_request.pose_stamped = PoseStamped()
request.ik_request.pose_stamped.header.frame_id = "base_link"
request.ik_request.pose_stamped.pose.position.x = 0.5
request.ik_request.pose_stamped.pose.orientation.w = 1.0

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)
print(future.result())
```

### Check Robot Model

```bash
# Verify URDF is valid
ros2 run moveit_core check_urdf < robot.urdf

# View robot model
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
