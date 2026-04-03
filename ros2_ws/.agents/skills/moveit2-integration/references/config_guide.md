# MoveIt2 Configuration Guide

Reference guide for MoveIt2 configuration files and their purposes.

## Table of Contents

- [kinematics.yaml](#kinematicsyaml)
- [joint_limits.yaml](#joint_limitsyaml)
- [ompl_planning.yaml](#ompl_planningyaml)
- [moveit_controllers.yaml](#moveit_controllersyaml)
- [robot.srdf](#robotsrdf)

---

## kinematics.yaml

Configures the Inverse Kinematics (IK) solver for each planning group.

```yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
```

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| `kinematics_solver` | IK solver plugin | `kdl_kinematics_plugin/KDLKinematicsPlugin` |
| `kinematics_solver_search_resolution` | Search step size | 0.005 rad |
| `kinematics_solver_timeout` | Max solve time | 0.005 s |
| `kinematics_solver_attempts` | Retry attempts | 3 |

### Available Solvers

- **KDLKinematicsPlugin**: General purpose, works with most robots
- **LMAKinematicsPlugin**: Faster for some robots, requires analytic IK
- **CachedKinematicsPlugin**: Wraps other solvers with caching
- **TRAC-IK**: Alternative solver with better success rate

---

## joint_limits.yaml

Defines velocity, acceleration, and position limits for each joint.

```yaml
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 2.618
    has_acceleration_limits: true
    max_acceleration: 8.0
    has_position_limits: true
    min_position: -3.14159
    max_position: 3.14159
```

### Important Notes

- Limits should match or be more conservative than hardware limits
- Setting acceleration limits too high causes trajectory rejection
- Setting velocity limits too low slows execution
- Use `has_*_limits: false` to disable specific limit checking

### Calculating from Spec Sheets

Convert RPM to rad/s:
```
max_velocity_rad_s = max_velocity_rpm * 2 * PI / 60
```

---

## ompl_planning.yaml

Configures the Open Motion Planning Library (OMPL) planners.

```yaml
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
start_state_max_bounds_error: 0.1
arm:
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
  projection_evaluator: joints(joint1,joint2)
  longest_valid_segment_fraction: 0.005
```

### Common Planner Configs

| Config | Description | Best For |
|--------|-------------|----------|
| RRTConnectkConfigDefault | Bidirectional RRT | Most planning tasks |
| RRTstar | Optimal RRT | High quality paths |
| PRMkConfigDefault | Probabilistic Roadmap | Multi-query planning |
| ESTkConfigDefault | Expansive Space Trees | Narrow passages |

### Request Adapters

- **AddTimeOptimalParameterization**: Adds time parameterization to path
- **ResolveConstraintFrames**: Resolves frame names in constraints
- **FixStartStateBounds**: Adjusts start state within bounds
- **FixStartStateCollision**: Attempts to fix start state in collision

---

## moveit_controllers.yaml

Maps MoveIt2 trajectory execution to ROS2 controllers.

```yaml
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller

  joint_trajectory_controller:
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
```

### Controller Types

| Type | Description |
|------|-------------|
| `FollowJointTrajectory` | Standard joint trajectory execution |
| `GripperCommand` | Parallel gripper control |
| `GripperAction` | Generic gripper action |

### Important Notes

- `action_ns` must match the controller's action namespace
- `default: true` makes this controller the default for the group
- Joints list must match the controller's configured joints

---

## robot.srdf

Semantic Robot Description Format - defines planning groups, poses, and semantics.

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Planning group -->
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0" />
  </group>
  
  <!-- Named configuration -->
  <group_state name="home" group="arm">
    <joint name="joint1" value="0" />
    <joint name="joint2" value="-1.57" />
    <joint name="joint3" value="0" />
    <joint name="joint4" value="-1.57" />
    <joint name="joint5" value="0" />
    <joint name="joint6" value="0" />
  </group_state>
  
  <!-- End effector -->
  <end_effector name="gripper" parent_link="tool0" group="gripper" />
  
  <!-- Disable collision between adjacent links -->
  <disable_collisions link1="base_link" link2="link1" />
</robot>
```

### SRDF Elements

| Element | Purpose |
|---------|---------|
| `<group>` | Define a planning group (chain or joint collection) |
| `<group_state>` | Named joint configuration (e.g., "home", "ready") |
| `<end_effector>` | Define end effector for the group |
| `<disable_collisions>` | Disable collision checking between specific links |

### Group Definition Patterns

**Chain pattern** (most common):
```xml
<group name="arm">
  <chain base_link="base_link" tip_link="ee_link" />
</group>
```

**Joint collection**:
```xml
<group name="arm">
  <joint name="joint1" />
  <joint name="joint2" />
  <joint name="joint3" />
</group>
```

**Subgroup**:
```xml
<group name="whole_body">
  <group name="arm" />
  <group name="torso" />
</group>
```
