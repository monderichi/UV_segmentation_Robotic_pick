---
name: ros2-launch-analyzer
description: Analyze ROS2 Python launch files to understand the running system, including nodes, parameters, launch arguments, included launch files, and component interactions. Use when Kimi needs to understand what a launch file starts, how nodes are configured, what topics/services are involved, or the relationship between C++ nodes, Python scripts, MoveIt2, Gazebo, and other ROS2 components.
---

# ROS2 Launch File Analyzer

Analyze and understand ROS2 Python launch files to extract system architecture, node configurations, and component relationships.

## When to Use This Skill

Use this skill when you need to:
- Understand what nodes a launch file starts
- Identify launch arguments and their default values
- Find included/sub-launch files
- Map parameter configurations for nodes
- Trace topic/service connections between nodes
- Understand MoveIt2 + Gazebo integration patterns
- Analyze the relationship between C++ nodes and Python scripts

## Quick Start

### Analyzing a Launch File

1. **Read the launch file** to understand its structure
2. **Identify the pattern** used (see Common Patterns below)
3. **Use the helper script** to extract structured information:
   ```bash
   python3 scripts/analyze_launch.py <path_to_launch_file>
   ```
4. **Trace included launch files** recursively
5. **Map nodes to their source files** (C++ or Python)

### Key Launch File Components

| Component | Import Path | Purpose |
|-----------|-------------|---------|
| `LaunchDescription` | `from launch import LaunchDescription` | Container for launch actions |
| `DeclareLaunchArgument` | `from launch.actions import DeclareLaunchArgument` | Define CLI arguments |
| `Node` | `from launch_ros.actions import Node` | Start a ROS2 node |
| `IncludeLaunchDescription` | `from launch.actions import IncludeLaunchDescription` | Include other launch files |
| `OpaqueFunction` | `from launch.actions import OpaqueFunction` | Deferred evaluation |
| `TimerAction` | `from launch.actions import TimerAction` | Delayed execution |
| `ExecuteProcess` | `from launch.actions import ExecuteProcess` | Run shell commands |
| `IfCondition`/`UnlessCondition` | `from launch.conditions import ...` | Conditional launching |

## Common Launch File Patterns

### Pattern 1: Direct Launch Description

Simple launch file with immediate node definitions:

```python
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arg_name", default_value="default"),
        Node(
            package="pkg_name",
            executable="node_name",
            parameters=[{"param": "value"}]
        ),
    ])
```

**Analysis approach**: Direct parsing - all nodes and arguments are defined in `generate_launch_description()`.

### Pattern 2: OpaqueFunction with Context

Deferred configuration using `OpaqueFunction`:

```python
def launch_setup(context, *args, **kwargs):
    # Access launch arguments via context
    arg_value = LaunchConfiguration("arg_name").perform(context)
    # ... create nodes based on context
    return [node1, node2, ...]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arg_name"),
        OpaqueFunction(function=launch_setup),
    ])
```

**Analysis approach**: 
- Arguments are resolved at runtime via `LaunchConfiguration().perform(context)`
- Nodes are created in `launch_setup()` based on argument values
- Look for `get_package_share_directory()` calls for config file paths

### Pattern 3: MoveIt2 + Gazebo Integration

Complex launch with robot simulation and motion planning:

```python
def launch_setup(context, *args, **kwargs):
    # Robot description from XACRO
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("pkg"), "urdf", "file.xacro"]),
        # ... xacro arguments
    ])
    
    # Gazebo spawn
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "name", "-topic", "robot_description"],
    )
    
    # MoveIt move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[robot_description, robot_description_semantic, ...],
    )
```

**Analysis approach**:
- See [references/moveit_launch_patterns.md](references/moveit_launch_patterns.md) for detailed MoveIt2 patterns
- See [references/gazebo_ros_patterns.md](references/gazebo_ros_patterns.md) for Gazebo integration

### Pattern 4: Including Other Launch Files

```python
included_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare("pkg"), "launch", "file.launch.py"])
    ]),
    launch_arguments={"arg": "value"}.items(),
    condition=IfCondition(some_condition),  # Optional
)
```

**Analysis approach**:
- Trace the included file path
- Note argument overrides in `launch_arguments`
- Check for conditional inclusion

## Extracting Node Information

### From a Node Definition

```python
Node(
    package="pkg_name",           # ROS package
    executable="exec_name",        # Node executable (from CMakeLists.txt setup.py)
    name="node_name",              # ROS node name (optional, overrides default)
    output="screen",               # Log output destination
    parameters=[                   # Parameter dictionary
        {"use_sim_time": True},
        config_file_path,          # Can reference YAML files
    ],
    arguments=["--arg1", "val"],   # Command line arguments
    remappings=[                   # Topic remappings
        ("old_topic", "new_topic"),
    ],
    condition=IfCondition(...),    # Conditional launch
)
```

### Finding the Source Code

1. **Python nodes**: Look in `package_name/scripts/` or `package_name/package_name/`
2. **C++ nodes**: Check `CMakeLists.txt` for `add_executable()` and `install()` targets
3. **Entry points**: For Python, check `setup.py` or `setup.cfg` for console scripts

## Analyzing Parameter Configurations

### YAML File Loading

```python
import yaml
from ament_index_python.packages import get_package_share_directory
import os

config_path = os.path.join(
    get_package_share_directory('pkg_name'),
    'config',
    'config.yaml'
)
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)
```

### Common MoveIt2 Config Files

| File | Purpose | Location |
|------|---------|----------|
| `kinematics.yaml` | IK solver settings | `moveit_config/config/` |
| `joint_limits.yaml` | Joint velocity/acceleration limits | `moveit_config/config/` |
| `ompl_planning.yaml` | OMPL planner configuration | `moveit_config/config/` |
| `moveit_controllers.yaml` | Controller manager setup | `moveit_config/config/` |
| `*.srdf` / `*.srdf.xacro` | Semantic robot description | `moveit_config/srdf/` |

## Tracing Topic Connections

### Common Topic Patterns

| Topic Pattern | Publisher | Subscriber | Purpose |
|--------------|-----------|------------|---------|
| `/robot_description` | `robot_state_publisher` | Gazebo, RViz, MoveIt | URDF string |
| `/joint_states` | Gazebo/Real robot | `robot_state_publisher`, MoveIt | Joint positions |
| `/tf`, `/tf_static` | `robot_state_publisher` | Various | Transform tree |
| `/clock` | Gazebo | All with `use_sim_time` | Simulation time |
| `/points`, `/depth/points` | Camera sensors | Processing nodes | Point cloud data |

### Controller Topics

Controllers loaded via `controller_manager`:
- `/controller_manager` service for loading/unloading
- `/joint_trajectory_controller/follow_joint_trajectory` action for motion

## Analysis Workflow

1. **Identify launch file pattern** (Direct, OpaqueFunction, or Mixed)
2. **Extract launch arguments** and their defaults
3. **List all Node actions** with package/executable names
4. **Trace included launch files** recursively
5. **Map parameters** to their source files
6. **Identify conditions** that affect node launching
7. **Find the source** for each node (C++ or Python)

## Reference Documentation

- **MoveIt2 Launch Patterns**: See [references/moveit_launch_patterns.md](references/moveit_launch_patterns.md)
- **Gazebo ROS Patterns**: See [references/gazebo_ros_patterns.md](references/gazebo_ros_patterns.md)
- **Common Node Types**: See [references/common_node_types.md](references/common_node_types.md)

## Example Analysis

### Sample: Analyzing bringup_v4.launch.py

```python
# 1. Pattern: OpaqueFunction with context
# 2. Key arguments:
#    - ur_type: ur10e
#    - description_file: my_robot.urdf.xacro
#    - world_file: table_world.world
# 3. Nodes launched:
#    - robot_state_publisher (from robot_description)
#    - joint_state_broadcaster (controller_manager)
#    - initial_joint_controller (controller_manager)
#    - spawn_ur (gazebo_ros spawn_entity.py)
# 4. Included launches:
#    - ur_moveit_launch (MoveIt configuration)
#    - gzserver, gzclient (Gazebo)
# 5. TimerAction: Runs transform_camera_pointcloud.py after 10s delay
```

Use the `analyze_launch.py` script to get structured output from any launch file.
