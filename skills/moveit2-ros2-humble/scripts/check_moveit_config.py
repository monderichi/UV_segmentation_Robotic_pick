#!/usr/bin/env python3
"""
Check MoveIt2 configuration for common issues.
Usage: python3 check_moveit_config.py <package_name>
"""

import sys
import os
import yaml
from pathlib import Path


def find_package_path(package_name):
    """Find ROS2 package path"""
    result = os.popen(f'ros2 pkg prefix {package_name}').read().strip()
    if not result:
        return None
    share_path = os.path.join(result, 'share', package_name)
    if os.path.exists(share_path):
        return share_path
    return None


def check_file_exists(pkg_path, file_path, description):
    """Check if a file exists"""
    full_path = os.path.join(pkg_path, file_path)
    if os.path.exists(full_path):
        print(f"  ✓ {description}: {file_path}")
        return True
    else:
        print(f"  ✗ {description}: {file_path} NOT FOUND")
        return False


def check_yaml_valid(file_path):
    """Check if YAML file is valid"""
    try:
        with open(file_path, 'r') as f:
            yaml.safe_load(f)
        return True
    except Exception as e:
        print(f"    ⚠ YAML Error: {e}")
        return False


def check_srdf(srdf_path):
    """Check SRDF file for common issues"""
    with open(srdf_path, 'r') as f:
        content = f.read()
    
    issues = []
    
    # Check for required elements
    if '<group ' not in content:
        issues.append("No planning groups defined")
    if '<virtual_joint' not in content and 'fixed' not in content:
        issues.append("No virtual joint defined (may be needed for mobile bases)")
    if '<disable_collisions' not in content:
        issues.append("No collision matrix defined (will be slow)")
    
    if issues:
        for issue in issues:
            print(f"    ⚠ SRDF: {issue}")
    else:
        print(f"    ✓ SRDF structure looks good")


def check_kinematics(kinematics_path):
    """Check kinematics.yaml"""
    with open(kinematics_path, 'r') as f:
        config = yaml.safe_load(f)
    
    for group_name, group_config in config.items():
        if 'kinematics_solver' not in group_config:
            print(f"    ⚠ Group '{group_name}': No kinematics_solver defined")
        else:
            print(f"    ✓ Group '{group_name}': {group_config['kinematics_solver']}")


def check_controllers(controllers_path):
    """Check ros2_controllers.yaml"""
    with open(controllers_path, 'r') as f:
        config = yaml.safe_load(f)
    
    if 'controller_manager' not in config:
        print("    ⚠ No controller_manager configuration")
        return
    
    cm_config = config.get('controller_manager', {}).get('ros__parameters', {})
    
    # Check for required controllers
    required = ['joint_state_broadcaster']
    for ctrl in required:
        if ctrl in cm_config:
            print(f"    ✓ {ctrl} defined")
        else:
            print(f"    ⚠ {ctrl} not defined")
    
    # Check for trajectory controller
    traj_ctrls = [k for k in cm_config.keys() 
                  if 'trajectory' in cm_config[k].get('type', '')]
    if traj_ctrls:
        print(f"    ✓ Trajectory controllers: {', '.join(traj_ctrls)}")
    else:
        print("    ⚠ No trajectory controller found")


def main():
    if len(sys.argv) < 2:
        print("Usage: check_moveit_config.py <package_name>")
        sys.exit(1)
    
    package_name = sys.argv[1]
    
    print(f"Checking MoveIt configuration for: {package_name}\n")
    
    pkg_path = find_package_path(package_name)
    if not pkg_path:
        print(f"Error: Package '{package_name}' not found")
        print("Make sure the package is built and sourced")
        sys.exit(1)
    
    print(f"Package path: {pkg_path}\n")
    
    # Check required files
    print("Required files:")
    check_file_exists(pkg_path, 'config/kinematics.yaml', 'Kinematics config')
    check_file_exists(pkg_path, 'config/joint_limits.yaml', 'Joint limits')
    check_file_exists(pkg_path, 'config/ros2_controllers.yaml', 'Controllers')
    check_file_exists(pkg_path, 'config/moveit_controllers.yaml', 'MoveIt controllers')
    check_file_exists(pkg_path, 'config/ompl_planning.yaml', 'OMPL planning')
    
    # Check SRDF
    srdf_found = False
    for srdf_path in ['config/robot.srdf', f'config/{package_name}.srdf']:
        if check_file_exists(pkg_path, srdf_path, 'SRDF'):
            check_srdf(os.path.join(pkg_path, srdf_path))
            srdf_found = True
            break
    if not srdf_found:
        print("  ✗ SRDF: No SRDF file found")
    
    print("\nChecking file contents:")
    
    # Check kinematics
    kin_path = os.path.join(pkg_path, 'config/kinematics.yaml')
    if os.path.exists(kin_path):
        print("\nKinematics configuration:")
        if check_yaml_valid(kin_path):
            check_kinematics(kin_path)
    
    # Check controllers
    ctrl_path = os.path.join(pkg_path, 'config/ros2_controllers.yaml')
    if os.path.exists(ctrl_path):
        print("\nControllers configuration:")
        if check_yaml_valid(ctrl_path):
            check_controllers(ctrl_path)
    
    # Check launch files
    print("\nLaunch files:")
    launch_dir = os.path.join(pkg_path, 'launch')
    if os.path.exists(launch_dir):
        launch_files = list(Path(launch_dir).glob('*.py'))
        if launch_files:
            for f in launch_files:
                print(f"  ✓ {f.name}")
        else:
            print("  ⚠ No launch files found")
    else:
        print("  ✗ No launch directory")
    
    print("\n✓ Configuration check complete")


if __name__ == '__main__':
    main()
