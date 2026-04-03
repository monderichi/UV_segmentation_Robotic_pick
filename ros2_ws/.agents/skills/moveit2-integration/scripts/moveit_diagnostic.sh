#!/bin/bash
#
# MoveIt2 Diagnostic Script
# 
# Usage:
#   ./moveit_diagnostic.sh [package_name]
#
# This script performs comprehensive diagnostics on a MoveIt2 setup
# including URDF/SRDF validation, controller status, and TF tree checks.

set -e

PACKAGE_NAME=${1:-""}
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================"
echo "MoveIt2 Diagnostic Tool"
echo "========================================"
echo ""

# Check ROS2 environment
echo -e "${YELLOW}=== ROS2 Environment ===${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}ERROR: ROS2 not sourced!${NC}"
    echo "Please source ROS2: source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "ROS_DISTRO: $ROS_DISTRO"
echo ""

# Check if move_group is running
echo -e "${YELLOW}=== MoveIt Nodes ===${NC}"
if ros2 node list | grep -q "move_group"; then
    echo -e "${GREEN}✓ move_group is running${NC}"
    ros2 node info /move_group 2>/dev/null | head -5
else
    echo -e "${RED}✗ move_group is NOT running${NC}"
fi
echo ""

# Check robot description
echo -e "${YELLOW}=== Robot Description ===${NC}"
if ros2 topic list | grep -q "/robot_description"; then
    echo -e "${GREEN}✓ /robot_description topic exists${NC}"
    
    # Check if it's being published
    if ros2 topic info /robot_description 2>/dev/null | grep -q "Publishers: None"; then
        echo -e "${RED}✗ No publishers on /robot_description${NC}"
    else
        echo -e "${GREEN}✓ robot_description is being published${NC}"
    fi
else
    echo -e "${RED}✗ /robot_description topic not found${NC}"
fi
echo ""

# Check joint states
echo -e "${YELLOW}=== Joint States ===${NC}"
if ros2 topic list | grep -q "/joint_states"; then
    echo -e "${GREEN}✓ /joint_states topic exists${NC}"
    echo "Current joint states:"
    timeout 2 ros2 topic echo /joint_states --once 2>/dev/null | head -20 || echo "(no data received)"
else
    echo -e "${RED}✗ /joint_states topic not found${NC}"
fi
echo ""

# Check controllers
echo -e "${YELLOW}=== Controller Status ===${NC}"
if command -v ros2 &> /dev/null; then
    echo "Controllers:"
    ros2 control list_controllers 2>/dev/null || echo -e "${RED}Controller manager not available${NC}"
    
    echo ""
    echo "Hardware Interfaces:"
    ros2 control list_hardware_interfaces 2>/dev/null || echo "N/A"
else
    echo -e "${RED}ros2 command not found${NC}"
fi
echo ""

# Check TF tree
echo -e "${YELLOW}=== TF Tree ===${NC}"
if ros2 topic list | grep -q "/tf"; then
    echo -e "${GREEN}✓ /tf topic exists${NC}"
    
    # Check for common frames
    echo "Checking common frames..."
    timeout 2 ros2 run tf2_ros tf2_echo world base_link 2>/dev/null | head -5 || echo -e "${YELLOW}⚠ world -> base_link not available${NC}"
else
    echo -e "${RED}✗ /tf topic not found${NC}"
fi
echo ""

# Check MoveIt action servers
echo -e "${YELLOW}=== MoveIt Action Servers ===${NC}"
if ros2 action list 2>/dev/null | grep -q "move_group"; then
    echo -e "${GREEN}✓ move_group action servers found:${NC}"
    ros2 action list 2>/dev/null | grep "move_group" || true
else
    echo -e "${RED}✗ No move_group action servers found${NC}"
fi
echo ""

# Check for planning scene
echo -e "${YELLOW}=== Planning Scene ===${NC}"
if ros2 service list 2>/dev/null | grep -q "/get_planning_scene"; then
    echo -e "${GREEN}✓ /get_planning_scene service available${NC}"
else
    echo -e "${RED}✗ /get_planning_scene service not found${NC}"
fi
echo ""

# Package-specific checks
echo -e "${YELLOW}=== Package Structure ===${NC}"
if [ -n "$PACKAGE_NAME" ]; then
    if [ -d "src/$PACKAGE_NAME" ]; then
        PKG_PATH="src/$PACKAGE_NAME"
    elif ros2 pkg prefix "$PACKAGE_NAME" &>/dev/null; then
        PKG_PATH=$(ros2 pkg prefix "$PACKAGE_NAME")/share/$PACKAGE_NAME
    else
        echo -e "${RED}Package '$PACKAGE_NAME' not found${NC}"
        PKG_PATH=""
    fi
    
    if [ -n "$PKG_PATH" ]; then
        echo "Package path: $PKG_PATH"
        
        # Check for config files
        echo ""
        echo "Configuration files:"
        [ -f "$PKG_PATH/config/robot.srdf" ] && echo -e "${GREEN}✓${NC} SRDF found" || echo -e "${RED}✗${NC} SRDF not found"
        [ -f "$PKG_PATH/config/kinematics.yaml" ] && echo -e "${GREEN}✓${NC} kinematics.yaml found" || echo -e "${RED}✗${NC} kinematics.yaml not found"
        [ -f "$PKG_PATH/config/joint_limits.yaml" ] && echo -e "${GREEN}✓${NC} joint_limits.yaml found" || echo -e "${YELLOW}⚠${NC} joint_limits.yaml not found"
        [ -f "$PKG_PATH/config/ompl_planning.yaml" ] && echo -e "${GREEN}✓${NC} ompl_planning.yaml found" || echo -e "${YELLOW}⚠${NC} ompl_planning.yaml not found"
        [ -f "$PKG_PATH/config/moveit_controllers.yaml" ] && echo -e "${GREEN}✓${NC} moveit_controllers.yaml found" || echo -e "${YELLOW}⚠${NC} moveit_controllers.yaml not found"
        
        # Check URDF
        echo ""
        echo "Robot description:"
        if [ -f "$PKG_PATH/urdf/robot.urdf.xacro" ]; then
            echo -e "${GREEN}✓${NC} robot.urdf.xacro found"
        elif ls "$PKG_PATH/urdf/"*.xacro 1> /dev/null 2>&1; then
            echo -e "${GREEN}✓${NC} Xacro files found:"
            ls "$PKG_PATH/urdf/"*.xacro 2>/dev/null | head -5
        else
            echo -e "${RED}✗${NC} No URDF/Xacro files found"
        fi
        
        # Check launch files
        echo ""
        echo "Launch files:"
        if [ -d "$PKG_PATH/launch" ]; then
            ls "$PKG_PATH/launch/"*.py 2>/dev/null | xargs -n1 basename || echo "(none)"
        else
            echo -e "${RED}✗${NC} No launch directory"
        fi
    fi
else
    echo "No package specified. Usage: $0 [package_name]"
fi
echo ""

echo "========================================"
echo "Diagnostic complete"
echo "========================================"
