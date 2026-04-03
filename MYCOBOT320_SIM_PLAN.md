# myCobot 320 M5 Simulation Plan

## Overview
This document outlines the plan for creating a new Gazebo Sim (Ignition) simulation for the **myCobot 320 M5** robot, following the same pattern as the existing UR10e simulation.

## Key Files Created

### 1. Launch Script
**File:** `ros2_ws/launch_mycobot320_sim.sh`

A standalone bash launcher that:
- Cleans Snap environment variables (fixes GTK/GIO issues)
- Sources ROS2 Humble environment
- Builds workspace if needed
- Clears MoveIt warehouse database
- Launches the myCobot 320 simulation

**Usage:**
```bash
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws
./launch_mycobot320_sim.sh
```

### 2. Launch File
**File:** `ros2_ws/src/spraying_pathways/launch/bringup_mycobot_320_gz_sim.launch.py`

Main ROS2 launch file that:
- Uses Gazebo Sim (Ignition) instead of Gazebo Classic
- Spawns myCobot 320 M5 at position (0.25, 0, 0.715)
- Starts MoveIt2 with proper configuration
- Handles proper startup sequence with event handlers
- Supports optional `go_home_node` with delay
- Configurable GUI/headless mode

**Usage:**
```bash
# Full simulation with GUI
ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py

# Headless mode (no Gazebo GUI)
ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py gazebo_gui:=false

# With automatic homing after 15 seconds
ros2 launch spraying_pathways bringup_mycobot_320_gz_sim.launch.py run_go_home:=true
```

### 3. World File
**File:** `ros2_ws/src/spraying_pathways/worlds/mycobot_320_world.sdf`

Gazebo Sim world with:
- Ground plane
- Directional lighting (sun)
- Work table at appropriate height
- Physics configuration (1ms step)

## UR-Compatible Version (For C++ Scripts)

To ensure compatibility with existing UR10e C++ scripts, I've created a **UR-compatible version** that uses standard UR joint and link names.

### Name Mapping

| myCobot Original | UR-Compatible | Description |
|-----------------|---------------|-------------|
| `joint2_to_joint1` | `shoulder_pan_joint` | Base rotation |
| `joint3_to_joint2` | `shoulder_lift_joint` | Shoulder |
| `joint4_to_joint3` | `elbow_joint` | Elbow |
| `joint5_to_joint4` | `wrist_1_joint` | Wrist 1 |
| `joint6_to_joint5` | `wrist_2_joint` | Wrist 2 |
| `joint6output_to_joint6` | `wrist_3_joint` | Wrist 3 |
| `link1` | `shoulder_link` | Shoulder link |
| `link2` | `upper_arm_link` | Upper arm |
| `link3` | `forearm_link` | Forearm |
| `link4` | `wrist_1_link` | Wrist 1 link |
| `link5` | `wrist_2_link` | Wrist 2 link |
| `link6` | `wrist_3_link` | Wrist 3 link |
| - | `flange` | Flange frame (UR standard) |
| - | `tool0` | Tool frame (UR standard) |

### UR-Compatible Files

#### Robot Description
**File:** `ros2_ws/src/mycobot_description/urdf/robots/mycobot_320_ur_compatible.urdf.xacro`

This xacro file wraps the original mycobot with UR-standard naming:
- Creates parallel UR-named link/joint structure
- Uses fixed joints to connect UR names to internal mycobot links
- Includes `flange` and `tool0` frames (standard UR frames)

**File:** `ros2_ws/src/mycobot_description/urdf/control/mycobot_320_ur_ros2_control.urdf.xacro`

ROS2 control configuration with UR joint names for Gazebo Sim.

#### Config Files
**Directory:** `ros2_ws/src/spraying_pathways/config/mycobot_320_ur/`

| File | Purpose |
|------|---------|
| `mycobot_320_ur.srdf` | Semantic description with UR names |
| `kinematics.yaml` | KDL solver config |
| `joint_limits.yaml` | Joint limits with UR names |
| `ros2_controllers.yaml` | Controller config with UR joint names |
| `moveit_controllers.yaml` | MoveIt controller mapping |
| `initial_positions.yaml` | Initial joint positions |
| `ompl_planning.yaml` | OMPL planner configuration |

### Using the UR-Compatible Version

The launch file `bringup_mycobot_320_gz_sim.launch.py` **automatically uses the UR-compatible version** by default.

## Architecture Comparison

| Component | UR10e (Existing) | myCobot 320 M5 (New) |
|-----------|------------------|----------------------|
| Launch script | `launch_gazebo_sim.sh` | `launch_mycobot320_sim.sh` |
| Launch file | `bringup_v4_gz_sim.launch.py` | `bringup_mycobot_320_gz_sim.launch.py` |
| Description pkg | `ur_description` | `mycobot_description` (source) |
| Controller name | `joint_trajectory_controller` | `arm_controller` |
| Joints | 6 (UR naming) | 6 (UR-compatible naming) |
| World file | `empty_world_gz_sim.sdf` | `mycobot_320_world.sdf` |
| MoveIt config | `ur_moveit_config` | `spraying_pathways/config/mycobot_320_ur` |

## Startup Sequence

```
1. Set Gazebo resource paths
2. Start Gazebo Sim (GUI or headless)
3. Start /clock bridge
4. Start robot_state_publisher
5. [DELAY 5s] Spawn robot in Gazebo
6. [ON SPAWN EXIT] Start controllers:
   - joint_state_broadcaster
   - arm_controller
7. [ON JSB EXIT] Start MoveIt move_group
8. [OPTIONAL] Start RViz
9. [OPTIONAL] Run go_home_node after delay
```

## Dependencies

Required ROS2 packages:
- `ros_gz_sim` - Gazebo Sim integration
- `ros_gz_bridge` - Topic bridge
- `gz_ros2_control` - ROS2 control for Gazebo Sim
- `mycobot_description` - Robot description (from source)
- `moveit_ros_move_group` - MoveIt planning
- `joint_state_broadcaster` - Joint state publishing
- `joint_trajectory_controller` - Trajectory execution

## Building

```bash
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws

# Build mycobot_description and spraying_pathways
colcon build --packages-select mycobot_description spraying_pathways

# Source the workspace
source install/setup.bash
```

## Testing Checklist

- [ ] Build workspace: `colcon build --packages-select mycobot_description spraying_pathways`
- [ ] Run launch script: `./launch_mycobot320_sim.sh`
- [ ] Verify robot spawns correctly in Gazebo
- [ ] Check joint states are published on `/joint_states`
- [ ] Verify controllers are active: `ros2 control list_controllers`
- [ ] Test MoveIt planning in RViz
- [ ] Test go_home_node execution
- [ ] Test headless mode: `gazebo_gui:=false`

## Running C++ Scripts

With the UR-compatible configuration, you can run existing C++ scripts:

```bash
# Go home (same as UR10e)
ros2 run spraying_pathways go_home_node

# Cartesian path planning
ros2 run spraying_pathways cartesian_path_planner_cubes_v4_node

# Other scripts
ros2 run spraying_pathways flat_fan_spraying_v4_node
ros2 run spraying_pathways lidar_surface_scanner_node
```

## Notes

1. **Existing UR10e simulation is preserved** - All original files remain unchanged
2. **Parallel simulations** - Both UR10e and myCobot 320 can coexist
3. **Gazebo Sim vs Classic** - Uses newer Ignition-based Gazebo Sim
4. **UR Compatibility** - Joint/link names match UR standard for script compatibility
5. **MoveIt integration** - Inline MoveIt configuration with `ur_manipulator` planning group

## Troubleshooting

### Issue: Robot doesn't spawn
**Solution:** Check Gazebo resource path includes mycobot_description meshes

### Issue: Controllers fail to start
**Solution:** Verify `arm_controller` is listed in `ros2_controllers.yaml`

### Issue: MoveIt can't plan
**Solution:** Check that SRDF file exists and joint names match (`shoulder_pan_joint`, etc.)

### Issue: RViz shows robot in wrong position
**Solution:** Verify `initial_positions.yaml` matches spawn position

### Issue: C++ scripts don't work
**Solution:** Ensure you're using the UR-compatible config (`mycobot_320_ur` directory)
