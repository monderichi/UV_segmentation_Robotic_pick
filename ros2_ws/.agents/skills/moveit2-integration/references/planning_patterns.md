# MoveIt2 Planning Patterns

Common motion planning patterns and code examples.

## Table of Contents

- [Basic Planning](#basic-planning)
- [Pose Goals](#pose-goals)
- [Cartesian Paths](#cartesian-paths)
- [Constraints](#constraints)
- [Planning Scene Operations](#planning-scene-operations)

---

## Basic Planning

### Plan to Named Target

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="moveit_py")
arm = moveit.get_planning_component("arm")

arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="home")

plan_result = arm.plan()
if plan_result:
    arm.execute()
```

### Plan to Joint Values

```python
from moveit.core.robot_state import RobotState

robot_model = moveit.get_robot_model()
goal_state = RobotState(robot_model)
goal_state.joint_positions = {
    "joint1": 0.5,
    "joint2": -0.5,
    "joint3": 1.0,
    "joint4": 0.0,
    "joint5": 0.5,
    "joint6": 0.0
}

arm.set_goal_state(robot_state=goal_state)
plan_result = arm.plan()
```

### Asynchronous Planning

```python
import threading

def plan_async():
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    result = arm.plan()
    if result:
        arm.execute()

thread = threading.Thread(target=plan_async)
thread.start()
# Do other work...
thread.join()
```

---

## Pose Goals

### Plan to Cartesian Pose

```python
from geometry_msgs.msg import PoseStamped

goal_pose = PoseStamped()
goal_pose.header.frame_id = "base_link"
goal_pose.pose.position.x = 0.5
goal_pose.pose.position.y = 0.0
goal_pose.pose.position.z = 0.3
goal_pose.pose.orientation.x = 0.0
goal_pose.pose.orientation.y = 1.0
goal_pose.pose.orientation.z = 0.0
goal_pose.pose.orientation.w = 0.0

arm.set_goal_state(pose_stamped=goal_pose, pose_link="tool0")
plan_result = arm.plan()
```

### Relative Motion

```python
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped

# Get current pose
current_pose = arm.get_current_pose()

# Create relative transform
transform = TransformStamped()
transform.transform.translation.x = 0.1  # Move 10cm in X
transform.transform.rotation.w = 1.0

# Apply transform
new_pose = tf2_geometry_msgs.do_transform_pose(current_pose, transform)

arm.set_goal_state(pose_stamped=new_pose, pose_link="tool0")
plan_result = arm.plan()
```

---

## Cartesian Paths

### Linear Path with Waypoints

```python
from geometry_msgs.msg import PoseStamped
import numpy as np

waypoints = []
current_pose = arm.get_current_pose()

# Create straight line waypoints
for i in range(10):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = current_pose.pose.position.x + (i * 0.01)
    pose.pose.position.y = current_pose.pose.position.y
    pose.pose.position.z = current_pose.pose.position.z
    pose.pose.orientation = current_pose.pose.orientation
    waypoints.append(pose)

# Plan Cartesian path
fraction, trajectory = arm.plan_cartesian_path(
    waypoints,
    eef_step=0.01,
    jump_threshold=0.0
)

if fraction > 0.9:  # At least 90% success
    arm.execute(trajectory)
```

### Circular Path

```python
def create_circle_waypoints(center, radius, num_points=20):
    waypoints = []
    for i in range(num_points + 1):
        angle = 2 * np.pi * i / num_points
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = center[0] + radius * np.cos(angle)
        pose.pose.position.y = center[1] + radius * np.sin(angle)
        pose.pose.position.z = center[2]
        # Keep orientation pointing at center
        pose.pose.orientation.w = 1.0
        waypoints.append(pose)
    return waypoints

waypoints = create_circle_waypoints([0.5, 0.0, 0.3], 0.1)
fraction, trajectory = arm.plan_cartesian_path(waypoints)
```

---

## Constraints

### Joint Constraints

```python
from moveit.core.kinematic_constraints import construct_joint_constraint

# Constrain specific joints
joint_constraint = construct_joint_constraint(
    robot_state=start_state,
    joint_model_group=robot_model.get_joint_model_group("arm"),
    tolerance_above=[0.1] * 6,
    tolerance_below=[0.1] * 6
)

arm.set_path_constraints(joint_constraint)
arm.plan()
arm.clear_path_constraints()
```

### Orientation Constraints

```python
from moveit_msgs.msg import Constraints, OrientationConstraint

orientation_constraint = OrientationConstraint()
orientation_constraint.link_name = "tool0"
orientation_constraint.header.frame_id = "base_link"
orientation_constraint.orientation.x = 0.0
orientation_constraint.orientation.y = 1.0
orientation_constraint.orientation.z = 0.0
orientation_constraint.orientation.w = 0.0
orientation_constraint.absolute_x_axis_tolerance = 0.1
orientation_constraint.absolute_y_axis_tolerance = 0.1
orientation_constraint.absolute_z_axis_tolerance = 0.1
orientation_constraint.weight = 1.0

constraints = Constraints()
constraints.orientation_constraints.append(orientation_constraint)

arm.set_path_constraints(constraints)
```

### Position Constraints

```python
from moveit_msgs.msg import PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

position_constraint = PositionConstraint()
position_constraint.link_name = "tool0"
position_constraint.header.frame_id = "base_link"

# Define constraint region (box)
box = SolidPrimitive()
box.type = SolidPrimitive.BOX
box.dimensions = [0.1, 0.1, 0.1]

constraint_region = BoundingVolume()
constraint_region.primitives.append(box)
constraint_region.primitive_poses.append(constraint_pose)

position_constraint.constraint_region = constraint_region
position_constraint.weight = 1.0
```

---

## Planning Scene Operations

### Add Collision Box

```python
from moveit.core.planning_scene import PlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

planning_scene = PlanningScene()

box = SolidPrimitive()
box.type = SolidPrimitive.BOX
box.dimensions = [0.2, 0.2, 0.5]

box_pose = Pose()
box_pose.position.x = 0.5
box_pose.position.y = 0.3
box_pose.position.z = 0.25
box_pose.orientation.w = 1.0

planning_scene.add_object("obstacle", box, box_pose)
```

### Add Collision Mesh

```python
from moveit.core.planning_scene import PlanningScene

planning_scene = PlanningScene()
planning_scene.add_mesh_object(
    name="part",
    mesh_file="package://my_pkg/meshes/part.stl",
    pose=mesh_pose
)
```

### Attach Object to Robot

```python
from moveit.core.planning_scene import PlanningScene

planning_scene = PlanningScene()
planning_scene.attach_object(
    object_name="gripped_object",
    link_name="tool0",
    touch_links=["tool0", "gripper_left", "gripper_right"]
)
```

### Remove Object

```python
from moveit.core.planning_scene import PlanningScene

planning_scene = PlanningScene()
planning_scene.remove_object("obstacle")
```

### Check Collision

```python
from moveit.core.planning_scene import PlanningScene
from moveit.core.collision_detection import CollisionRequest, CollisionResult

request = CollisionRequest()
request.group_name = "arm"
result = CollisionResult()

planning_scene.check_collision(request, result, current_state)
if result.collision:
    print("Collision detected!")
```
