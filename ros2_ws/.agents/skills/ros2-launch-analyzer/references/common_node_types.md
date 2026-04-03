# Common Node Types in ROS2 Robotics Projects

Reference for frequently encountered nodes in MoveIt2 + Gazebo + C++/Python ROS2 projects.

---

## Core ROS2 Nodes

### robot_state_publisher

Publishes the robot's TF tree based on URDF and joint states.

```python
Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[
        {"use_sim_time": True},
        robot_description,
    ],
)
```

**Subscribes to:** `/joint_states`
**Publishes to:** `/tf`, `/tf_static`, `/robot_description`

---

### joint_state_publisher_gui

GUI for manually controlling joint positions.

```python
Node(
    package="joint_state_publisher_gui",
    executable="joint_state_publisher_gui",
    parameters=[robot_description, {"use_sim_time": use_sim_time}],
)
```

**Publishes to:** `/joint_states`

---

## Gazebo Nodes

### spawn_entity.py

Spawns a robot/model into Gazebo simulation.

```python
Node(
    package="gazebo_ros",
    executable="spawn_entity.py",
    arguments=[
        "-entity", "robot_name",
        "-topic", "robot_description",
        "-x", "0.0", "-y", "0.0", "-z", "0.0",
    ],
    output="screen",
)
```

**Note:** This is a one-time execution node.

---

## Controller Manager Nodes

### spawner (joint_state_broadcaster)

Broadcasts joint states from hardware/simulation.

```python
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
)
```

**Publishes to:** `/joint_states`

### spawner (joint_trajectory_controller)

Executes joint trajectory commands.

```python
Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
)
```

**Action Server:** `/joint_trajectory_controller/follow_joint_trajectory`

---

## MoveIt2 Nodes

### move_group

Core MoveIt planning and execution node.

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
        {"use_sim_time": True},
    ],
)
```

**Action Servers:**
- `/move_group` - Main planning interface
- `/execute_trajectory` - Trajectory execution

**Services:**
- `/plan_kinematic_path` - Path planning
- `/compute_cartesian_path` - Cartesian planning
- `/check_state_validity` - State validation

---

### moveit_servo

Real-time Cartesian control (optional).

```python
Node(
    package="moveit_servo",
    executable="servo_node_main",
    parameters=[servo_config],
)
```

---

## Visualization Nodes

### rviz2

3D visualization tool.

```python
Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", path_to_rviz_config],
    parameters=[
        robot_description,
        robot_description_semantic,
        {"use_sim_time": True},
    ],
)
```

---

## Common C++ Node Patterns

### Cartesian Path Planner

Nodes that plan and execute Cartesian paths using MoveIt.

**Typical pattern:**
```cpp
// Subscribes to target pose topics
// Uses MoveGroupInterface for planning
// Publishes trajectory execution status
```

**Common parameters:**
- `planning_group`: Name of the MoveIt planning group
- `eef_frame`: End-effector link name
- `velocity_scaling`: Velocity scaling factor (0.0-1.0)
- `acceleration_scaling`: Acceleration scaling factor

---

### Point Cloud Processor

Processes sensor point cloud data.

**Typical subscriptions:**
- `/camera/points` or `/depth/points` - Raw point cloud
- `/tf` or `/tf_static` - For transformations

**Typical publications:**
- `/processed_points` - Filtered/transformed cloud
- `/marker_array` - Visualization markers

**Common parameters:**
- `frame_id`: Target frame for transformation
- `voxel_size`: Downsampling voxel size
- `distance_threshold`: Filtering distance

---

### Safety Monitor

Monitors system state and triggers safety stops.

**Typical subscriptions:**
- `/joint_states` - Joint positions
- `/camera/points` - Environment sensing
- `/tf` - Object positions

**Typical publications:**
- `/safety_stop` - Emergency stop command
- `/safety_status` - Safety system status

---

## Common Python Node Patterns

### Transform Node

Transforms point clouds or poses between frames.

**Typical pattern:**
```python
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class TransformNode(Node):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Subscribe to source topic, transform, publish to target topic
```

---

### Obstacle Detector

Detects obstacles from sensor data.

**Typical subscriptions:**
- `/camera/points` - Point cloud
- `/lidar/scan` - Laser scan (alternative)

**Typical publications:**
- `/obstacles` - Detected obstacle markers
- `/collision_objects` - MoveIt collision objects

---

## HRI Safety Nodes

### safety_stop_on_unknown

Stops robot when unknown objects are detected.

```python
Node(
    package="spraying_pathways",
    executable="safety_stop_on_unknown_node",
    parameters=[{"safety_distance": 0.5}],
)
```

**Typical subscriptions:**
- `/unknown_points` - Unknown object point cloud
- `/joint_states` - Current robot state

**Typical publications:**
- `/safety_stop` - Stop command topic

---

## Spraying/Path Planning Nodes

### flat_fan_spraying

Controls spraying end-effector along paths.

**Typical subscriptions:**
- `/planned_path` - Path to follow
- `/joint_states` - Current position

**Typical publications:**
- `/spray_command` - Spray on/off commands
- `/spray_status` - Current spraying state

---

### lidar_surface_scanner

Scans surfaces using LIDAR for path planning.

**Typical subscriptions:**
- `/scan` or `/lidar/points`
- `/tf` - For sensor pose

**Typical publications:**
- `/surface_points` - Detected surface points
- `/surface_normals` - Surface normal vectors

---

## Custom Node Identification

### Finding Node Source

When you see a Node in a launch file:

```python
Node(
    package="spraying_pathways",
    executable="cartesian_path_planner_cubes_v4_node",
)
```

**To find source:**

1. **For C++ nodes:** Check `CMakeLists.txt`:
   ```cmake
   add_executable(cartesian_path_planner_cubes_v4_node src/cartesian_path_planner_cubes_v4.cpp)
   ```

2. **For Python nodes:** Check `setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'script_name = package.module:main',
       ],
   }
   ```

---

## Node Communication Patterns

### MoveIt + Gazebo Data Flow

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Gazebo Sim    │────▶│ joint_state_     │────▶│ robot_state_    │
│   (Physics)     │     │ broadcaster      │     │ publisher       │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
         │                                               │
         │                                               ▼
         │                                        ┌──────────────┐
         │                                        │     /tf      │
         │                                        └──────┬───────┘
         │                                               │
         │         ┌─────────────────┐                   │
         └────────▶│  move_group     │◀──────────────────┘
                   │  (MoveIt)       │
                   └────────┬────────┘
                            │
                            ▼
                   ┌─────────────────┐
                   │ joint_trajectory_│
                   │ controller      │
                   └────────┬────────┘
                            │
                            ▼
                   ┌─────────────────┐
                   │   Gazebo Sim    │
                   │   (Execution)   │
                   └─────────────────┘
```

### Sensor Processing Pipeline

```
┌─────────────┐    ┌─────────────────────┐    ┌─────────────────┐
│   Camera    │───▶│  transform_node     │───▶│  filter_node    │
│  /points    │    │  (TF transform)     │    │  (processing)   │
└─────────────┘    └─────────────────────┘    └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │  planner_node   │
                                              │  (MoveIt client)│
                                              └────────┬────────┘
                                                       │
                                                       ▼
                                              ┌─────────────────┐
                                              │  move_group     │
                                              └─────────────────┘
```

---

## Topic Naming Conventions

| Prefix | Purpose | Example |
|--------|---------|---------|
| `/` | Global topics | `/tf`, `/clock` |
| `/robot_name/` | Robot-specific | `/ur/joint_states` |
| `/camera/` | Camera data | `/camera/points` |
| `/lidar/` | LIDAR data | `/lidar/scan` |
| `/move_group/` | MoveIt internal | `/move_group/feedback` |
| `/controller_name/` | Controller data | `/joint_trajectory_controller/state` |
