#!/usr/bin/env python3
"""
MoveIt2 helper script for common operations.

Usage:
    python3 moveit_helper.py --plan --group arm --joint-values "0.0,0.0,0.0,0.0,0.0,0.0"
    python3 moveit_helper.py --plan --group arm --pose "0.5,0.0,0.3,0.0,1.0,0.0,0.0"
    python3 moveit_helper.py --list-groups
    python3 moveit_helper.py --get-current-state --group arm
"""

import argparse
import sys
import rclpy
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped


def parse_joint_values(joint_values_str: str) -> dict:
    """Parse comma-separated joint values into a dictionary."""
    values = [float(v) for v in joint_values_str.split(",")]
    return {f"joint{i+1}": v for i, v in enumerate(values)}


def parse_pose(pose_str: str) -> PoseStamped:
    """Parse comma-separated pose values into PoseStamped."""
    values = [float(v) for v in pose_str.split(",")]
    if len(values) != 7:
        raise ValueError("Pose must have 7 values: x,y,z,qx,qy,qz,qw")
    
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = values[0]
    pose.pose.position.y = values[1]
    pose.pose.position.z = values[2]
    pose.pose.orientation.x = values[3]
    pose.pose.orientation.y = values[4]
    pose.pose.orientation.z = values[5]
    pose.pose.orientation.w = values[6]
    return pose


def list_groups(moveit: MoveItPy):
    """List all available planning groups."""
    print("Available planning groups:")
    robot_model = moveit.get_robot_model()
    for group_name in robot_model.get_joint_model_group_names():
        group = robot_model.get_joint_model_group(group_name)
        joint_names = group.get_joint_model_names()
        print(f"  - {group_name}: {joint_names}")


def get_current_state(moveit: MoveItPy, group_name: str):
    """Get current joint state for a planning group."""
    arm = moveit.get_planning_component(group_name)
    robot_state = arm.get_start_state()
    
    print(f"Current state for group '{group_name}':")
    robot_model = moveit.get_robot_model()
    group = robot_model.get_joint_model_group(group_name)
    
    for joint_name in group.get_joint_model_names():
        joint_state = robot_state.get_joint_positions(joint_name)
        print(f"  {joint_name}: {joint_state}")


def plan_to_joints(moveit: MoveItPy, group_name: str, joint_values: dict):
    """Plan to specific joint values."""
    arm = moveit.get_planning_component(group_name)
    
    robot_model = moveit.get_robot_model()
    goal_state = RobotState(robot_model)
    
    for joint_name, value in joint_values.items():
        goal_state.set_joint_positions(joint_name, [value])
    
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)
    
    print(f"Planning to joint values: {joint_values}")
    plan_result = arm.plan()
    
    if plan_result:
        print("Planning succeeded!")
        return plan_result
    else:
        print("Planning failed!")
        return None


def plan_to_pose(moveit: MoveItPy, group_name: str, pose: PoseStamped, link_name: str = None):
    """Plan to a Cartesian pose."""
    arm = moveit.get_planning_component(group_name)
    
    if link_name is None:
        # Try to determine end effector link
        robot_model = moveit.get_robot_model()
        group = robot_model.get_joint_model_group(group_name)
        link_name = group.get_link_model_names()[-1]
    
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped=pose, pose_link=link_name)
    
    print(f"Planning to pose: {pose.pose}")
    plan_result = arm.plan()
    
    if plan_result:
        print("Planning succeeded!")
        return plan_result
    else:
        print("Planning failed!")
        return None


def plan_to_named_target(moveit: MoveItPy, group_name: str, target_name: str):
    """Plan to a named target from SRDF."""
    arm = moveit.get_planning_component(group_name)
    
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name=target_name)
    
    print(f"Planning to named target: {target_name}")
    plan_result = arm.plan()
    
    if plan_result:
        print("Planning succeeded!")
        return plan_result
    else:
        print("Planning failed!")
        return None


def main():
    parser = argparse.ArgumentParser(description="MoveIt2 helper script")
    parser.add_argument("--group", default="arm", help="Planning group name")
    parser.add_argument("--list-groups", action="store_true", help="List available planning groups")
    parser.add_argument("--get-current-state", action="store_true", help="Get current joint state")
    parser.add_argument("--plan", action="store_true", help="Plan motion")
    parser.add_argument("--execute", action="store_true", help="Execute plan after planning")
    parser.add_argument("--joint-values", help="Target joint values (comma-separated)")
    parser.add_argument("--pose", help="Target pose x,y,z,qx,qy,qz,qw (comma-separated)")
    parser.add_argument("--named-target", help="Named target from SRDF")
    parser.add_argument("--end-effector", help="End effector link name for pose goals")
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        moveit = MoveItPy(node_name="moveit_helper")
        
        if args.list_groups:
            list_groups(moveit)
            return 0
        
        if args.get_current_state:
            get_current_state(moveit, args.group)
            return 0
        
        if args.plan:
            plan_result = None
            
            if args.joint_values:
                joint_values = parse_joint_values(args.joint_values)
                plan_result = plan_to_joints(moveit, args.group, joint_values)
            elif args.pose:
                pose = parse_pose(args.pose)
                plan_result = plan_to_pose(moveit, args.group, pose, args.end_effector)
            elif args.named_target:
                plan_result = plan_to_named_target(moveit, args.group, args.named_target)
            else:
                print("Error: Must specify --joint-values, --pose, or --named-target for planning")
                return 1
            
            if plan_result and args.execute:
                arm = moveit.get_planning_component(args.group)
                print("Executing plan...")
                arm.execute()
                print("Execution complete!")
        else:
            print("No action specified. Use --help for usage information.")
            return 1
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        rclpy.shutdown()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
