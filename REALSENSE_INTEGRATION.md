# myCobot 320 M5 + RealSense D455 Integration

This setup connects a RealSense D455 camera on a separate stand to the myCobot 320 M5 in MoveIt2.

Supports both **Simulation (Gazebo)** and **Real Robot** modes.

## Setup Overview

```
                    Camera Stand (D455)
                           |
                           | 30cm above
                           v
    [Camera] --------> [Workspace]
       |                    |
       | 20cm X offset       | Robot working
       v                    v
    [Captures]          [Robot Arm]
    point cloud         myCobot 320 M5
```

## Files Created/Modified

| File | Purpose |
|------|---------|
| `ros2_ws/src/mycobot_description/urdf/sensors/realsense_d455.urdf.xacro` | D455 Camera URDF |
| `ros2_ws/src/spraying_pathways/launch/bringup_mycobot_320_realsense.launch.py` | **Main launch file** (sim + real robot) |
| `ros2_ws/launch_mycobot320_realsense.sh` | **Main launcher script** |

## Quick Start

### Simulation Mode (Default)

```bash
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws
./launch_mycobot320_realsense.sh
```

### Real Robot Mode

```bash
./launch_mycobot320_realsense.sh --real
```

With custom serial port:

```bash
./launch_mycobot320_realsense.sh --real --port /dev/ttyUSB0
```

### Custom Camera Position (Real Robot)

```bash
./launch_mycobot320_realsense.sh --real --x 0.6 --y 0.0 --z 0.55
```

## Usage Options

### Launcher Script Options

```bash
./launch_mycobot320_realsense.sh [options]

Options:
  --real          Run with REAL robot (no Gazebo simulation)
  --sim           Run with Gazebo simulation (default)
  --no-camera     Don't launch RealSense camera
  --port PORT     Serial port for real robot (default: /dev/ttyACM0)
  --x <value>     Camera X position (default: 0.5)
  --y <value>     Camera Y position (default: 0.0)
  --z <value>     Camera Z position (default: 0.5)
  -h, --help      Show help message
```

### Direct ROS2 Launch

```bash
cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws
source install/setup.bash

# Simulation + camera
ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py

# Real robot + camera
ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py use_gazebo:=false

# Real robot with custom port and camera position
ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py \
  use_gazebo:=false \
  robot_port:=/dev/ttyUSB0 \
  camera_x:=0.6 \
  camera_y:=0.0 \
  camera_z:=0.55

# Without camera
ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py run_realsense:=false
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_gazebo` | `true` | Use Gazebo simulation (`false` = real robot) |
| `robot_port` | `/dev/ttyACM0` | Serial port for real robot |
| `robot_baud` | `115200` | Baud rate for real robot |
| `spray_port` | `/dev/ttyACM1` | Serial port for spray valve Arduino |
| `run_realsense` | `true` | Launch RealSense camera |
| `camera_x` | `0.5` | Camera X position relative to base_link |
| `camera_y` | `0.0` | Camera Y position relative to base_link |
| `camera_z` | `0.5` | Camera Z position relative to base_link |
| `camera_roll` | `3.14159` | Camera roll (PI = upside down) |
| `camera_pitch` | `0.0` | Camera pitch |
| `camera_yaw` | `0.0` | Camera yaw |

## Real Robot Safety

When using `--real` mode, the launcher will:

1. **Display safety checklist**
2. **Check serial port exists**
3. **Fix port permissions** (if needed)
4. **Start the hardware driver** (`mycobot_driver.py`)

**⚠️ SAFETY CHECKLIST:**
- Ensure robot has CLEAR WORKSPACE
- Emergency stop button is ACCESSIBLE
- Robot is powered ON
- USB cable is connected

## Camera Position

The camera is positioned on a stand relative to the robot base (`base_link`):
- **Default X**: 0.5m (20cm offset from typical work position)
- **Default Y**: 0.0m
- **Default Z**: 0.5m (30cm above work position)
- **Orientation**: Top-down (upside down, roll=180°)

Measure your actual camera position and adjust with `--x`, `--y`, `--z` options.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB image (1280x720 @ 30fps) |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | Depth image |
| `/camera/depth/color/points` | sensor_msgs/PointCloud2 | Colorized point cloud |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Camera calibration |
| `/joint_states` | sensor_msgs/JointState | Robot joint positions |

## Visualizing in RViz

1. **Start the system**: Run the launcher script
2. **Wait for startup**: ~15 seconds for camera to initialize
3. **In RViz**:
   - Fixed Frame: `base_link`
   - Add displays:
     - **RobotModel** - Shows the robot
     - **TF** - Shows frames including `camera_link`
     - **PointCloud2** - Topic: `/camera/depth/color/points`
     - **Image** - Topic: `/camera/color/image_raw`

## Point Cloud in MoveIt

The launch file configures MoveIt to use the point cloud for collision checking:
- **Octomap resolution**: 1cm voxels
- **Max range**: 2 meters
- **Topic**: `/camera/depth/color/points`

Objects detected in the point cloud will appear as collision objects in the MoveIt planning scene.

## Controlling the Real Robot

After launching with `--real`, you can control the robot using:

```bash
# Go to home position
ros2 run spraying_pathways go_home_node

# Run trajectory planner
ros2 run spraying_pathways cartesian_path_planner_trajectory_v1_mycobot320_node

# Monitor joint states
ros2 topic echo /joint_states

# Check robot connection
ros2 topic echo /joint_states | head -5
```

## Startup Sequences

### Simulation Mode (~25s total)
1. Gazebo Sim starts (8s)
2. Robot spawns (8s delay)
3. Controllers activate: JSB → arm_controller
4. MoveIt move_group starts
5. RViz opens
6. RealSense camera starts (15s)

### Real Robot Mode (~15s total)
1. Robot driver starts (connects to hardware)
2. MoveIt move_group starts (5s delay)
3. RViz opens
4. RealSense camera starts (15s)

## Troubleshooting

### Serial port not found
```bash
# List available ports
ls -la /dev/tty*

# Find robot port
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Fix permissions
sudo chmod 666 /dev/ttyACM0
```

### RViz crashes
The launcher automatically cleans Snap environment variables. If RViz still crashes:
```bash
unset GTK_PATH GTK_MODULES GIO_MODULE_DIR
export QT_QPA_PLATFORM=xcb
./launch_mycobot320_realsense.sh --real
```

### Camera not detected
```bash
# Check USB connection
lsusb | grep Intel

# Check video devices
ls -la /dev/video*
```

### Real robot not moving
1. Check driver is running: `ros2 node list | grep mycobot`
2. Check joint states are publishing: `ros2 topic echo /joint_states`
3. Check for errors in terminal output
4. Verify robot is powered on and USB connected

### No point cloud in RViz
1. Check topic is publishing: `ros2 topic hz /camera/depth/color/points`
2. Check TF frame exists: `ros2 run tf2_tools view_frames`
3. Verify "Fixed Frame" in RViz is set to `base_link`

## Hardware Requirements

- **Robot**: myCobot 320 M5
- **Camera**: RealSense D455 (USB 3.0)
- **USB Cables**: USB 3.0 for camera, USB for robot
- **Camera Mount**: Tripod or stand
- **Computer**: ROS2 Humble compatible

## Software Requirements

```bash
# Install RealSense ROS2 package
sudo apt install ros-humble-realsense2-camera

# Install pymycobot for real robot control
pip3 install pymycobot

# Fix serial port permissions (one-time)
sudo usermod -a -G dialout $USER
# Log out and log back in after this
```

## D455 vs D456

The D455 and D456 are nearly identical:
- Same sensors and optics
- D456 adds IP65 dust/water protection
- Same ROS2 driver and topics

This integration works with both cameras.

## Next Steps

1. **Calibrate camera position**: Use ArUco markers to fine-tune the static transform
2. **Filter point cloud**: Remove table/ground plane to isolate objects
3. **Object detection**: Integrate YOLO or similar
4. **Grasp planning**: Use point cloud for collision-aware planning
