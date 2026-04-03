#!/bin/bash
# Check RealSense D456 connection and configuration

echo "=========================================="
echo "RealSense D456 Connection Checker"
echo "=========================================="
echo ""

# Check if realsense2_camera package is installed
echo "[1] Checking realsense2_camera package..."
if ros2 pkg list | grep -q realsense2_camera; then
    echo "  ✓ realsense2_camera package found"
    ros2 pkg prefix realsense2_camera
else
    echo "  ✗ realsense2_camera package NOT found!"
    echo "  Install with: sudo apt install ros-humble-realsense2-camera"
    exit 1
fi
echo ""

# Check if camera is connected via USB
echo "[2] Checking USB devices..."
if lsusb | grep -i "Intel" | grep -i "RealSense"; then
    echo "  ✓ RealSense device detected via USB"
else
    echo "  ⚠ No RealSense device found in USB list"
    echo "  Check USB connection and power"
fi
echo ""

# Check video devices
echo "[3] Checking video devices..."
if ls -la /dev/video* 2>/dev/null | grep -q video; then
    echo "  Video devices found:"
    ls -la /dev/video*
else
    echo "  ⚠ No video devices found"
fi
echo ""

# Check ROS2 topics after running camera
echo "[4] To check camera topics, run:"
echo "    ros2 topic list | grep camera"
echo ""

echo "[5] To view camera streams:"
echo "    ros2 run rqt_image_view rqt_image_view"
echo ""

echo "[6] To echo point cloud (first 5 messages):"
echo "    ros2 topic echo /camera/depth/color/points --once"
echo ""

echo "=========================================="
echo "Launch Commands"
echo "=========================================="
echo ""
echo "1. Launch with camera:"
echo "   cd /media/monder/Files/robotics/project_phee/ros2_moveit_docker/ros2_ws"
echo "   source install/setup.bash"
echo "   ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py"
echo ""
echo "2. Adjust camera position (example):"
echo "   ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py \\"
echo "     camera_x:=0.6 camera_y:=0.1 camera_z:=0.7"
echo ""
echo "3. Launch simulation WITHOUT camera (for testing):"
echo "   ros2 launch spraying_pathways bringup_mycobot_320_realsense.launch.py \\"
echo "     run_realsense:=false"
echo ""
echo "4. Launch just the camera:"
echo "   ros2 launch spraying_pathways realsense_d456_stand.launch.py"
echo ""
