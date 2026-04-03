# MoveIt2 Launch Patterns

Common patterns for launching MoveIt2 motion planning systems in ROS2.

## Table of Contents

1. [Basic MoveIt2 Launch Structure](#basic-moveit2-launch-structure)
2. [MoveIt Configs Builder Pattern](#moveit-configs-builder-pattern)
3. [Manual Configuration Pattern](#manual-configuration-pattern)
4. [Common Nodes](#common-nodes)
5. [Configuration Files](#configuration-files)
6. [Example: Complete Launch File](#example-complete-launch-file)

---

## Basic MoveIt2 Launch Structure

A typical MoveIt2 launch file creates these components:

1. **Robot description** (URDF/XACRO)
2. **Robot semantic description** (SRDF)
3. **MoveIt configuration** (kinematics, joint limits, planning)
4. **move_group node** (core planning node)
5. **RViz node** (visualization)
6. **Robot State Publisher** (TF tree)

---

## MoveIt Configs Builder Pattern

Using `moveit_configs_utils` for simplified configuration:

```python
from moveit_configs_utils import MoveItConfigsBuilder

moveit_config = (
    MoveItConfigsBuilder("robot_name", package_name="moveit_config_pkg")
    .robot_description(file_path=os.path.join(urdf_path, "robot.urdf.xacro"))
    .robot_description_semantic(file_path=os.path.join(config_path, "robot.srdf"))
    .trajectory_execution(file_path=os.path.join(config_path, "moveit_controllers.yaml"))
    .planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner"],
        default_planning_pipeline="ompl"
    )
    .to_moveit_configs()
)
```

### Builder Methods

| Method | Purpose | Config File |
|--------|---------|-------------|
| `.robot_description()` | Load URDF/XACRO | `*.urdf.xacro` |
| `.robot_description_semantic()` | Load SRDF | `*.srdf` or `*.srdf.xacro` |
| `.trajectory_execution()` | Controller setup | `moveit_controllers.yaml` |
| `.planning_pipelines()` | Planning algorithms | Auto-generated |
| `.robot_description_kinematics()` | IK solver config | `kinematics.yaml` |
| `.joint_limits()` | Joint constraints | `joint_limits.yaml` |
| `.planning_scene_monitor()` | Scene monitoring | Parameters dict |
| `.pilz_cartesian_limits()` | Cartesian limits | `pilz_cartesian_limits.yaml` |

---

## Manual Configuration Pattern

Direct parameter construction without the builder:

```python
# Load YAML configs
import yaml
from ament_index_python.packages import get_package_share_directory

moveit_config_pkg = get_package_share_directory('ur_moveit_config')

with open(os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml'), 'r') as f:
    kinematics = yaml.safe_load(f)

with open(os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml'), 'r') as f:
    joint_limits = yaml.safe_load(f)

with open(os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml'), 'r') as f:
    ompl = yaml.safe_load(f)

# Construct parameter dictionaries
robot_description_kinematics = {
    "robot_description_kinematics": kinematics.get("/**", {})
                                               .get("ros__parameters", {})
                                               .get("robot_description_kinematics", {})
}

robot_description_planning = {
    "robot_description_planning": joint_limits
}

ompl_config = {
    "move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
        **ompl,
    }
}

moveit_controllers = {
    "moveit_simple_controller_manager": controllers,
    "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
}

trajectory_execution = {
    "moveit_manage_controllers": False,
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
    "trajectory_execution.execution_duration_monitoring": False,
}

planning_scene_monitor_parameters = {
    "publish_planning_scene": True,
    "publish_geometry_updates": True,
    "publish_state_updates": True,
    "publish_transforms_updates": True,
}

# Create move_group node
move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_config,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor_parameters,
        {"use_sim_time": True},
    ],
)
```

---

## Common Nodes

### move_group Node

The core MoveIt planning node:

```python
Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        ompl_config,
        moveit_controllers,
        trajectory_execution,
        planning_scene_monitor_parameters,
        {"use_sim_time": True},
    ],
)
```

**Key Parameters:**
- `robot_description`: URDF string
- `robot_description_semantic`: SRDF string
- `robot_description_kinematics`: IK solver configuration
- `robot_description_planning`: Joint limits
- `move_group`: Planning pipeline configuration

### RViz Node

Visualization with MoveIt configuration:

```python
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2_moveit",
    output="log",
    arguments=["-d", os.path.join(moveit_config_pkg, "rviz", "view_robot.rviz")],
    parameters=[
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        robot_description_planning,
        {"use_sim_time": True},
    ],
)
```

---

## Configuration Files

### kinematics.yaml

Inverse kinematics solver configuration:

```yaml
/**:
  ros__parameters:
    robot_description_kinematics:
      manipulator:
        kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
        kinematics_solver_search_resolution: 0.005
        kinematics_solver_timeout: 0.005
        kinematics_solver_attempts: 3
```

### joint_limits.yaml

Joint velocity/acceleration limits:

```yaml
joint_limits:
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 3.15
    has_acceleration_limits: true
    max_acceleration: 3.15
  # ... other joints
```

### ompl_planning.yaml

OMPL planner configuration:

```yaml
planner_configs:
  SBLkConfigDefault:
    type: geometric::SBL
    range: 0.0
  ESTkConfigDefault:
    type: geometric::EST
    range: 0.0
  # ... other planners

manipulator:
  default_planner_config: RRTConnectkConfigDefault
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - RRTConnectkConfigDefault
  projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
  longest_valid_segment_fraction: 0.005
```

### moveit_controllers.yaml

Controller manager configuration:

```yaml
controller_names:
  - joint_trajectory_controller

joint_trajectory_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    # ... other joints
```

---

## Example: Complete Launch File

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    
    # Get paths
    moveit_config_pkg = get_package_share_directory(f"{robot_name}_moveit_config")
    
    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=f"{robot_name}_moveit_config")
        .trajectory_execution(file_path=os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml'))
        .robot_description_semantic(file_path=os.path.join(moveit_config_pkg, 'config', f'{robot_name}.srdf'))
        .planning_scene_monitor(
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    
    # Create nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(moveit_config_pkg, "rviz", "view_robot.rviz")],
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )
    
    return [move_group_node, rviz_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_name", default_value="ur10e"),
        OpaqueFunction(function=launch_setup),
    ])
```

---

## Including ur_moveit.launch.py

The standard Universal Robots MoveIt launch:

```python
ur_moveit_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"])
    ]),
    launch_arguments={
        "ur_type": "ur10e",
        "safety_limits": "true",
        "description_package": "ur_description",
        "description_file": "ur.urdf.xacro",
        "moveit_config_package": "ur_moveit_config",
        "moveit_config_file": "ur.srdf.xacro",
        "use_sim_time": "true",
        "launch_rviz": "true",
        "use_fake_hardware": "true",
    }.items(),
)
```

**Key Arguments:**
- `ur_type`: Robot model (ur3e, ur5e, ur10e, etc.)
- `use_sim_time`: Use simulation clock
- `launch_rviz`: Start RViz
- `use_fake_hardware`: Use fake hardware interface (for simulation)
