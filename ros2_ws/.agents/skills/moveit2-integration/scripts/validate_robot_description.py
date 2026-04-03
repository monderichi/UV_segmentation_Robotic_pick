#!/usr/bin/env python3
"""
Robot Description Validator for MoveIt2

Validates URDF/Xacro and SRDF files for common issues.

Usage:
    python3 validate_robot_description.py /path/to/robot.urdf.xacro /path/to/robot.srdf
    python3 validate_robot_description.py --xacro /path/to/robot.urdf.xacro --srdf /path/to/robot.srdf
"""

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def print_section(title: str):
    print("\n" + "=" * 60)
    print(f" {title}")
    print("=" * 60)


def print_ok(msg: str):
    print(f"  ✓ {msg}")


def print_error(msg: str):
    print(f"  ✗ {msg}")


def print_warning(msg: str):
    print(f"  ⚠ {msg}")


def run_xacro(xacro_path: str) -> tuple[bool, str]:
    """Process xacro file and return URDF content."""
    try:
        result = subprocess.run(
            ["xacro", xacro_path],
            capture_output=True,
            text=True,
            timeout=30
        )
        if result.returncode == 0:
            return True, result.stdout
        else:
            return False, result.stderr
    except FileNotFoundError:
        return False, "xacro not found. Install ros-$ROS_DISTRO-xacro"
    except subprocess.TimeoutExpired:
        return False, "xacro processing timed out"
    except Exception as e:
        return False, str(e)


def validate_urdf(urdf_content: str) -> list[str]:
    """Validate URDF content and return list of issues."""
    issues = []
    
    try:
        root = ET.fromstring(urdf_content)
    except ET.ParseError as e:
        return [f"XML Parse Error: {e}"]
    
    # Check robot name
    robot_name = root.get('name')
    if not robot_name:
        issues.append("Robot has no name attribute")
    
    # Find all links and joints
    links = root.findall('.//link')
    joints = root.findall('.//joint')
    
    print(f"\n  Found {len(links)} links, {len(joints)} joints")
    
    # Check for required links
    link_names = {link.get('name') for link in links}
    joint_names = {joint.get('name') for joint in joints}
    
    if 'base_link' not in link_names:
        issues.append("Missing required 'base_link'")
    
    # Check joints for common issues
    for joint in joints:
        jname = joint.get('name', 'unnamed')
        jtype = joint.get('type', 'unknown')
        
        # Check for problematic characters in joint names
        if any(c in jname for c in ['-', '[', ']', '(', ')']):
            issues.append(f"Joint '{jname}' contains invalid characters (-[]())")
        
        # Check joint limits
        if jtype in ['revolute', 'prismatic']:
            limit = joint.find('limit')
            if limit is None:
                issues.append(f"Joint '{jname}' (type={jtype}) missing limits")
            else:
                lower = limit.get('lower')
                upper = limit.get('upper')
                effort = limit.get('effort')
                velocity = limit.get('velocity')
                
                if lower is None or upper is None:
                    issues.append(f"Joint '{jname}' missing position limits")
                if effort is None:
                    issues.append(f"Joint '{jname}' missing effort limit")
                if velocity is None:
                    issues.append(f"Joint '{jname}' missing velocity limit")
    
    # Check for transmission elements (needed for Gazebo)
    transmissions = root.findall('.//transmission')
    if transmissions:
        print_ok(f"Found {len(transmissions)} transmission elements")
    
    # Check for ros2_control
    ros2_control = root.findall('.//ros2_control')
    if ros2_control:
        print_ok(f"Found {len(ros2_control)} ros2_control element(s)")
        
        # Check hardware plugin
        for rc in ros2_control:
            hw = rc.find('.//hardware/plugin')
            if hw is not None:
                print_ok(f"Hardware plugin: {hw.text}")
    
    # Check for Gazebo plugin
    gazebo_plugins = root.findall('.//gazebo/plugin')
    if gazebo_plugins:
        print_ok(f"Found {len(gazebo_plugins)} Gazebo plugin(s)")
        for gp in gazebo_plugins:
            fname = gp.get('filename', 'unknown')
            print(f"    - {fname}")
    
    return issues


def validate_srdf(srdf_path: str, urdf_joint_names: set = None, urdf_link_names: set = None) -> list[str]:
    """Validate SRDF file."""
    issues = []
    
    try:
        tree = ET.parse(srdf_path)
        root = tree.getroot()
    except ET.ParseError as e:
        return [f"XML Parse Error: {e}"]
    except FileNotFoundError:
        return [f"File not found: {srdf_path}"]
    
    # Check robot name matches
    robot_name = root.get('name')
    print(f"\n  Robot name: {robot_name}")
    
    # Check virtual joint
    virtual_joints = root.findall('virtual_joint')
    if not virtual_joints:
        issues.append("Missing virtual_joint (required for MoveIt)")
    else:
        print_ok(f"Found {len(virtual_joints)} virtual joint(s)")
        for vj in virtual_joints:
            print(f"    - {vj.get('name')}: {vj.get('parent_frame')} -> {vj.get('child_link')}")
    
    # Check planning groups
    groups = root.findall('group')
    if not groups:
        issues.append("No planning groups defined")
    else:
        print_ok(f"Found {len(groups)} planning group(s)")
        for g in groups:
            gname = g.get('name', 'unnamed')
            chains = g.findall('chain')
            joints = g.findall('joint')
            subgroups = g.findall('group')
            print(f"    - {gname}: {len(chains)} chain(s), {len(joints)} joint(s), {len(subgroups)} subgroup(s)")
    
    # Check end effectors
    end_effectors = root.findall('end_effector')
    if end_effectors:
        print_ok(f"Found {len(end_effectors)} end effector(s)")
        for ee in end_effectors:
            print(f"    - {ee.get('name')}: {ee.get('group')} on {ee.get('parent_link')}")
    
    # Check named poses
    group_states = root.findall('group_state')
    if group_states:
        print_ok(f"Found {len(group_states)} named pose(s)")
    
    # Check collision disables
    disable_collisions = root.findall('disable_collisions')
    if disable_collisions:
        print_ok(f"Found {len(disable_collisions)} collision disable(s)")
    
    # Validate against URDF if provided
    if urdf_joint_names:
        srdf_joints = set()
        for gs in group_states:
            for j in gs.findall('joint'):
                srdf_joints.add(j.get('name'))
        
        invalid_joints = srdf_joints - urdf_joint_names
        if invalid_joints:
            issues.append(f"SRDF references invalid joints not in URDF: {invalid_joints}")
    
    return issues


def main():
    parser = argparse.ArgumentParser(description='Validate robot description files for MoveIt2')
    parser.add_argument('urdf', nargs='?', help='Path to URDF or Xacro file')
    parser.add_argument('srdf', nargs='?', help='Path to SRDF file')
    parser.add_argument('--xacro', help='Path to Xacro file')
    parser.add_argument('--srdf', dest='srdf_arg', help='Path to SRDF file')
    
    args = parser.parse_args()
    
    # Determine file paths
    urdf_path = args.xacro or args.urdf
    srdf_path = args.srdf_arg or args.srdf
    
    all_ok = True
    urdf_joint_names = None
    urdf_link_names = None
    
    # Validate URDF/Xacro
    if urdf_path:
        print_section("URDF/XACRO Validation")
        print(f"File: {urdf_path}")
        
        if not Path(urdf_path).exists():
            print_error(f"File not found: {urdf_path}")
            all_ok = False
        else:
            # Process xacro if needed
            if urdf_path.endswith('.xacro'):
                print("\n  Processing xacro...")
                success, result = run_xacro(urdf_path)
                if not success:
                    print_error(f"Xacro processing failed: {result}")
                    all_ok = False
                else:
                    print_ok("Xacro processed successfully")
                    urdf_content = result
            else:
                urdf_content = Path(urdf_path).read_text()
            
            if all_ok:
                issues = validate_urdf(urdf_content)
                
                # Extract joint/link names for SRDF validation
                root = ET.fromstring(urdf_content)
                urdf_joint_names = {j.get('name') for j in root.findall('.//joint')}
                urdf_link_names = {l.get('name') for l in root.findall('.//link')}
                
                if issues:
                    print("\n  Issues found:")
                    for issue in issues:
                        print_error(issue)
                    all_ok = False
                else:
                    print_ok("URDF validation passed")
    
    # Validate SRDF
    if srdf_path:
        print_section("SRDF Validation")
        print(f"File: {srdf_path}")
        
        if not Path(srdf_path).exists():
            print_error(f"File not found: {srdf_path}")
            all_ok = False
        else:
            issues = validate_srdf(srdf_path, urdf_joint_names, urdf_link_names)
            
            if issues:
                print("\n  Issues found:")
                for issue in issues:
                    print_error(issue)
                all_ok = False
            else:
                print_ok("SRDF validation passed")
    
    # Summary
    print_section("Summary")
    if all_ok:
        print_ok("All validations passed!")
        return 0
    else:
        print_error("Some validations failed. Please fix the issues above.")
        return 1


if __name__ == '__main__':
    sys.exit(main())
