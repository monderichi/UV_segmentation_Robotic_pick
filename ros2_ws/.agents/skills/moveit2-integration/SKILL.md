---
name: moveit2-integration
description: Integrate and work with MoveIt2 for ROS2 robot motion planning, including MoveGroup setup, motion planning, trajectory execution, collision scene management, and controller integration. Use when Kimi needs to implement robot motion planning, configure MoveIt2 packages, write planning scripts, debug trajectory execution, or integrate motion planning with ROS2 controllers and Gazebo simulation.
---

# MoveIt2 Integration

Integrate MoveIt2 for robot motion planning and trajectory execution in ROS2 environments.

## When to Use This Skill

Use this skill when you need to:
- Configure MoveIt2 packages and launch files
- Implement motion planning using the MoveGroup interface
- Set up collision objects and planning scenes
- Debug trajectory execution issues
- Integrate MoveIt2 with ROS2 controllers
- Plan and execute joint-space or Cartesian motions
- Handle motion planning constraints

## Quick Start

### Basic MoveGroup Interface

```python
import rclpy
from moveit.planning import MoveItPy
from moveit.core.kinematic_constraints import construct_joint_constraint

# Initialize
rclpy.init()
node = rclpy.create_node("moveit_example")
moveit = MoveItPy(node_name="moveit_py")

# Get planning component
arm = moveit.get_planning_component("arm")

# Set goal and plan
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")
plan_result = arm.plan()

# Execute
if plan_result:
    arm.execute()
```

### Using the Helper Script

```bash
python3 scripts/moveit_helper.py --plan --group arm --joint-values "0.0,0.0,0.0,0.0,0.0,0.0"
```

## Core Concepts

| Component | Purpose | Key Class/Tool |
|-----------|---------|----------------|
| MoveGroup | Primary planning interface | `moveit.planning.MoveItPy` |
| Planning Scene | Collision geometry | `moveit.core.planning_scene.PlanningScene` |
| Robot Model | Kinematic description | Robot URDF + SRDF |
| Controllers | Trajectory execution | `JointTrajectoryController` |
| Constraints | Motion restrictions | `moveit.core.kinematic_constraints` |

## Common Patterns

### Pattern 1: Joint Space Planning

Plan to specific joint angles:

```python
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="moveit_py")
arm = moveit.get_planning_component("arm")

# Set goal by joint values
robot_model = moveit.get_robot_model()
goal_state = RobotState(robot_model)
joint_values = {
    "joint1": 0.5,
    "joint2": 0.5,
    "joint3": -0.5,
    "joint4": 0.0,
    "joint5": 0.0,
    "joint6": 0.0
}
goal_state.joint_positions = joint_values

arm.set_goal_state(robot_state=goal_state)
plan = arm.plan()
```

### Pattern 2: Cartesian Path Planning

Plan linear end-effector motion:

```python
from geometry_msgs.msg import PoseStamped
import numpy as np

# Get current pose
arm = moveit.get_planning_component("arm")
arm.set_start_state_to_current_state()

# Define waypoints
waypoints = []
start_pose = arm.get_current_pose()
for t in np.linspace(0, 1, 10):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = start_pose.pose.position.x + t * 0.1
    pose.pose.position.y = start_pose.pose.position.y
    pose.pose.position.z = start_pose.pose.position.z
    pose.pose.orientation = start_pose.pose.orientation
    waypoints.append(pose)

# Plan Cartesian path
fraction = arm.plan_cartesian_path(waypoints)
```

### Pattern 3: Adding Collision Objects

Add objects to the planning scene:

```python
from moveit.core.planning_scene import PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

planning_scene = PlanningScene()

# Create box
box = SolidPrimitive()
box.type = SolidPrimitive.BOX
box.dimensions = [0.1, 0.1, 0.1]

# Set pose
box_pose = Pose()
box_pose.position.x = 0.5
box_pose.position.y = 0.0
box_pose.position.z = 0.5

# Add to scene
planning_scene.add_object("box", box, box_pose)
```

### Pattern 4: Named Target Planning

Use predefined configurations from SRDF:

```python
# Named targets defined in robot_name.srdf
arm.set_goal_state(configuration_name="home")
arm.set_goal_state(configuration_name="ready")
arm.set_goal_state(configuration_name="extended")

plan = arm.plan()
if plan:
    arm.execute()
```

## Configuration Files

### MoveIt2 Package Structure

```
moveit_config/
├── config/
│   ├── kinematics.yaml          # IK solver config
│   ├── joint_limits.yaml        # Velocity/acceleration limits
│   ├── ompl_planning.yaml       # Planner settings
│   ├── moveit_controllers.yaml  # Controller mappings
│   └── sensors_3d.yaml          # Sensor configuration
├── srdf/
│   └── robot.srdf               # Semantic robot description
└── launch/
    └── move_group.launch.py     # MoveGroup launch
```

### Key Configuration Parameters

See [references/config_guide.md](references/config_guide.md) for detailed explanations of each config file.

## Controller Integration

### Joint Trajectory Controller

MoveIt2 requires a `JointTrajectoryController` for execution:

```yaml
# controller_manager.yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
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
```

### Controller Activation Flow

1. Spawn robot in Gazebo/real hardware
2. Load controller_manager
3. Load and start joint_state_broadcaster
4. Load joint_trajectory_controller
5. MoveIt2 connects to action server: `/joint_trajectory_controller/follow_joint_trajectory`

## Debugging

### Common Issues

| Issue | Cause | Solution |
|-------|-------|----------|
| "No controller found" | Controller not loaded | Check controller_manager and yaml config |
| Planning fails | Invalid goal state | Check joint limits and self-collision |
| Trajectory rejected | Velocity/accel limits | Adjust in joint_limits.yaml |
| Execution hangs | Action server timeout | Verify controller is active |

### Validation Steps

1. **Check robot description loads:**
   ```bash
   ros2 topic echo /robot_description
   ```

2. **Verify joint states:**
   ```bash
   ros2 topic echo /joint_states
   ```

3. **Test controller:**
   ```bash
   ros2 control list_controllers
   ```

4. **Check MoveGroup action servers:**
   ```bash
   ros2 action list | grep move_group
   ```

## Reference Documentation

- **Configuration Guide**: See [references/config_guide.md](references/config_guide.md)
- **Common Planning Patterns**: See [references/planning_patterns.md](references/planning_patterns.md)
- **Troubleshooting Guide**: See [references/troubleshooting.md](references/troubleshooting.md)

## Example: Complete Motion Script

```python
#!/usr/bin/env python3
import rclpy
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    node = rclpy.create_node("example_motion")
    
    # Initialize MoveIt2
    moveit = MoveItPy(node_name="moveit_py")
    arm = moveit.get_planning_component("arm")
    
    # Plan to home
    node.get_logger().info("Planning to home...")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    
    plan_result = arm.plan()
    if plan_result:
        node.get_logger().info("Executing plan...")
        arm.execute()
    else:
        node.get_logger().error("Planning failed!")
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
