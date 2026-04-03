#!/bin/bash
# Create a MoveIt configuration package from templates
# Usage: template_robot_moveit_config.sh <robot_name> [<description_package>]

set -e

ROBOT_NAME="$1"
DESCRIPTION_PKG="${2:-${ROBOT_NAME}_description}"

if [ -z "$ROBOT_NAME" ]; then
    echo "Usage: template_robot_moveit_config.sh <robot_name> [<description_package>]"
    echo ""
    echo "Creates a MoveIt configuration package with standard file structure"
    exit 1
fi

PKG_NAME="${ROBOT_NAME}_moveit_config"

echo "Creating MoveIt config package: $PKG_NAME"
echo "Robot description package: $DESCRIPTION_PKG"
echo ""

# Create directory structure
mkdir -p "$PKG_NAME"/config
mkdir -p "$PKG_NAME"/launch
mkdir -p "$PKG_NAME"/rviz

# Create package.xml
cat > "$PKG_NAME"/package.xml << EOF
<?xml version="1.0"?>
<package format="3">
  <name>$PKG_NAME</name>
  <version>0.0.1</version>
  <description>MoveIt configuration for $ROBOT_NAME</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>moveit_ros_move_group</depend>
  <depend>moveit_simple_controller_manager</depend>
  <depend>moveit_kinematics</depend>
  <depend>moveit_planners_ompl</depend>
  <depend>moveit_ros_visualization</depend>
  <depend>joint_state_publisher</depend>
  <depend>robot_state_publisher</depend>
  <depend>xacro</depend>
  <depend>$DESCRIPTION_PKG</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake_python</build_type>
  </export>
</package>
EOF

# Create CMakeLists.txt
cat > "$PKG_NAME"/CMakeLists.txt << EOF
cmake_minimum_required(VERSION 3.8)
project($PKG_NAME)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch rviz
  DESTINATION share/\${PROJECT_NAME}
)

ament_package()
EOF

# Create kinematics.yaml
cat > "$PKG_NAME"/config/kinematics.yaml << EOF
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3
EOF

# Create joint_limits.yaml
cat > "$PKG_NAME"/config/joint_limits.yaml << EOF
joint_limits:
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  shoulder_lift_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  elbow_joint:
    has_velocity_limits: true
    max_velocity: 3.14
    has_acceleration_limits: true
    max_acceleration: 5.0
  wrist_1_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
  wrist_2_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
  wrist_3_joint:
    has_velocity_limits: true
    max_velocity: 6.28
    has_acceleration_limits: true
    max_acceleration: 10.0
EOF

# Create ros2_controllers.yaml
cat > "$PKG_NAME"/config/ros2_controllers.yaml << EOF
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    gains:
      shoulder_pan_joint: {p: 100.0, i: 0.0, d: 10.0}
      shoulder_lift_joint: {p: 100.0, i: 0.0, d: 10.0}
      elbow_joint: {p: 100.0, i: 0.0, d: 10.0}
      wrist_1_joint: {p: 50.0, i: 0.0, d: 5.0}
      wrist_2_joint: {p: 50.0, i: 0.0, d: 5.0}
      wrist_3_joint: {p: 50.0, i: 0.0, d: 5.0}
EOF

# Create moveit_controllers.yaml
cat > "$PKG_NAME"/config/moveit_controllers.yaml << EOF
moveit_simple_controller_manager:
  ros__parameters:
    controller_names:
      - arm_controller

    arm_controller:
      action_ns: follow_joint_trajectory
      type: FollowJointTrajectory
      default: true
      joints:
        - shoulder_pan_joint
        - shoulder_lift_joint
        - elbow_joint
        - wrist_1_joint
        - wrist_2_joint
        - wrist_3_joint
EOF

# Create ompl_planning.yaml
cat > "$PKG_NAME"/config/ompl_planning.yaml << EOF
planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/ResolveConstraintFrames
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints

start_state_max_bounds_error: 0.1
jiggle_fraction: 0.05

manipulator:
  planner_configs:
    - SBLkConfigDefault
    - ESTkConfigDefault
    - LBKPIECEkConfigDefault
    - BKPIECEkConfigDefault
    - KPIECEkConfigDefault
    - RRTkConfigDefault
    - RRTConnectkConfigDefault
    - RRTstarkConfigDefault
    - TRRTkConfigDefault
    - PRMkConfigDefault
    - PRMstarkConfigDefault
    - FMTkConfigDefault
    - BFMTkConfigDefault
    - PDSTkConfigDefault
    - STRIDEkConfigDefault
    - BiTRRTkConfigDefault
    - LBTRRTkConfigDefault
    - BiESTkConfigDefault
    - ProjESTkConfigDefault
    - LazyPRMkConfigDefault
    - LazyPRMstarkConfigDefault
    - SPARSkConfigDefault
    - SPARStwokConfigDefault
    - TrajOptDefault
  projection_evaluator: joints(shoulder_pan_joint,shoulder_lift_joint)
  longest_valid_segment_fraction: 0.005

planner_configs:
  SBLkConfigDefault:
    type: geometric::SBL
    range: 0.0  # Max motion added to tree. <=0: use v.estep
  ESTkConfigDefault:
    type: geometric::EST
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    goal_bias: 0.05  # When close to goal select goal, with this probability
  LBKPIECEkConfigDefault:
    type: geometric::LBKPIECE
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    border_fraction: 0.9  # Fraction of time focused on boarder
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction
  BKPIECEkConfigDefault:
    type: geometric::BKPIECE
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    border_fraction: 0.9  # Fraction of time focused on boarder
    failed_expansion_score_factor: 0.5  # When extending motion fails, score factor of the node
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction
  KPIECEkConfigDefault:
    type: geometric::KPIECE
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    goal_bias: 0.05  # When close to goal select goal, with this probability
    border_fraction: 0.9  # Fraction of time focused on boarder
    failed_expansion_score_factor: 0.5  # When extending motion fails, score factor of the node
    min_valid_path_fraction: 0.5  # Accept partially valid moves above fraction
  RRTkConfigDefault:
    type: geometric::RRT
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    goal_bias: 0.05  # When close to goal select goal, with this probability
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0  # Max motion added to tree. <=0: use v.estep
  RRTstarkConfigDefault:
    type: geometric::RRTstar
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    goal_bias: 0.05  # When close to goal select goal, with this probability
    delay_collision_checking: 1
  TRRTkConfigDefault:
    type: geometric::TRRT
    range: 0.0  # Max motion added to tree. <=0: use v.estep
    goal_bias: 0.05  # When close to goal select goal, with this probability
    max_states_failed: 10  # When to start increasing temp
    temp_change_factor: 2.0  # How much to increase or decrease temp
    min_temperature: 10e-10  # Lower limit of temp change
    init_temperature: 10e-6  # Initial temperature
    frountier_threshold: 0.0  # Dist new state to nearest neighbor to become frountier
    frountier_node_ratio: 0.1  # Fraction of time for frontier selection
    k_constant: 0.0  # Value used to normalize expression
  PRMkConfigDefault:
    type: geometric::PRM
    max_nearest_neighbors: 10  # Use k nearest neighbors
  PRMstarkConfigDefault:
    type: geometric::PRMstar
EOF

# Create SRDF template
cat > "$PKG_NAME"/config/"$ROBOT_NAME".srdf << EOF
<?xml version="1.0"?>
<robot name="$ROBOT_NAME">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>

  <group_state name="home" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.57"/>
    <joint name="elbow_joint" value="1.57"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>

  <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base_link"/>

  <!-- Add disable_collisions entries here -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
</robot>
EOF

# Create main launch file
cat > "$PKG_NAME"/launch/"$ROBOT_NAME"_moveit.launch.py << 'EOF'
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    moveit_pkg = FindPackageShare('ROBOT_NAME_moveit_config')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('DESCRIPTION_PKG'),
            'urdf', 'robots', 'ROBOT_NAME.urdf.xacro'
        ])
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # MoveIt move_group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            PathJoinSubstitution([moveit_pkg, 'config', 'kinematics.yaml']),
            PathJoinSubstitution([moveit_pkg, 'config', 'joint_limits.yaml']),
            PathJoinSubstitution([moveit_pkg, 'config', 'ompl_planning.yaml']),
            PathJoinSubstitution([moveit_pkg, 'config', 'moveit_controllers.yaml']),
            {
                'robot_description': robot_description_content,
                'robot_description_semantic': Command([
                    'cat ', PathJoinSubstitution([
                        moveit_pkg, 'config', 'ROBOT_NAME.srdf'
                    ])
                ]),
                'use_sim_time': use_sim_time
            }
        ],
        output='screen'
    )
    
    # RViz
    rviz_config = PathJoinSubstitution([moveit_pkg, 'rviz', 'moveit.rviz'])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('use_rviz', default='true'))
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        robot_state_publisher,
        move_group,
        rviz
    ])
EOF

# Replace placeholders in launch file
sed -i "s/ROBOT_NAME/$ROBOT_NAME/g" "$PKG_NAME"/launch/"$ROBOT_NAME"_moveit.launch.py
sed -i "s/DESCRIPTION_PKG/$DESCRIPTION_PKG/g" "$PKG_NAME"/launch/"$ROBOT_NAME"_moveit.launch.py

# Add missing import
sed -i 's/from launch.actions import (/from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IfCondition, RegisterEventHandler/' "$PKG_NAME"/launch/"$ROBOT_NAME"_moveit.launch.py

echo "✓ Created package: $PKG_NAME"
echo ""
echo "Next steps:"
echo "1. cd $PKG_NAME"
echo "2. Update the SRDF file (config/$ROBOT_NAME.srdf) with your robot's links"
echo "3. Update joint names in config/*.yaml files"
echo "4. colcon build --packages-select $PKG_NAME"
echo "5. ros2 launch $PKG_NAME ${ROBOT_NAME}_moveit.launch.py"
