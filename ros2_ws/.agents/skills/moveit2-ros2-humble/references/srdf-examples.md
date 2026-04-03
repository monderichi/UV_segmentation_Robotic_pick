# SRDF Configuration Examples

## Table of Contents
1. [Basic Configuration](#basic-configuration)
2. [End Effectors](#end-effectors)
3. [Complex Planning Groups](#complex-planning-groups)
4. [Collision Matrix](#collision-matrix)
5. [State Configuration](#state-configuration)

## Basic Configuration

### Simple 6-DOF Arm

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Main planning group -->
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <!-- Predefined states -->
  <group_state name="home" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.5708"/>
    <joint name="elbow_joint" value="1.5708"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <group_state name="up" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.5708"/>
    <joint name="elbow_joint" value="0"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <!-- Virtual joint to connect robot to world -->
  <virtual_joint name="fixed_base" type="fixed" 
                 parent_frame="world" child_link="base_link"/>
  
  <!-- Disable adjacent link collisions -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
</robot>
```

## End Effectors

### Gripper End Effector

```xml
<?xml version="1.0"?>
<robot name="robot_with_gripper">
  <!-- Main arm group -->
  <group name="manipulator">
    <chain base_link="base_link" tip_link="gripper_base"/>
  </group>
  
  <!-- Gripper group (for controlling fingers) -->
  <group name="gripper">
    <joint name="finger_joint1"/>
    <joint name="finger_joint2"/>
  </group>
  
  <!-- End effector definition -->
  <end_effector name="gripper_ee" group="gripper" 
                parent_link="gripper_base" parent_group="manipulator"/>
  
  <!-- Gripper states -->
  <group_state name="open" group="gripper">
    <joint name="finger_joint1" value="0.04"/>
    <joint name="finger_joint2" value="0.04"/>
  </group_state>
  
  <group_state name="closed" group="gripper">
    <joint name="finger_joint1" value="0.0"/>
    <joint name="finger_joint2" value="0.0"/>
  </group_state>
</robot>
```

### Multi-Finger Gripper with Joints

```xml
<robot name="shadow_hand">
  <!-- Manipulator group -->
  <group name="arm">
    <chain base_link="base_link" tip_link="wrist"/>
  </group>
  
  <!-- Hand groups -->
  <group name="hand">
    <chain base_link="wrist" tip_link="finger_tip"/>
  </group>
  
  <group name="thumb">
    <joint name="thumb_base_joint"/>
    <joint name="thumb_proximal_joint"/>
    <joint name="thumb_distal_joint"/>
  </group>
  
  <group name="index_finger">
    <joint name="index_base_joint"/>
    <joint name="index_proximal_joint"/>
    <joint name="index_distal_joint"/>
  </group>
  
  <group name="middle_finger">
    <joint name="middle_base_joint"/>
    <joint name="middle_proximal_joint"/>
    <joint name="middle_distal_joint"/>
  </group>
  
  <!-- End effectors for different grasp types -->
  <end_effector name="precision_grasp" group="hand" 
                parent_link="wrist" parent_group="arm"/>
  <end_effector name="power_grasp" group="hand" 
                parent_link="wrist" parent_group="arm"/>
</robot>
```

## Complex Planning Groups

### Multiple Subgroups

```xml
<robot name="dual_arm_robot">
  <!-- Left arm -->
  <group name="left_arm">
    <chain base_link="torso" tip_link="left_tool0"/>
  </group>
  
  <!-- Right arm -->
  <group name="right_arm">
    <chain base_link="torso" tip_link="right_tool0"/>
  </group>
  
  <!-- Combined both arms -->
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
  
  <!-- Left arm with torso -->
  <group name="left_arm_torso">
    <joint name="torso_joint"/>
    <chain base_link="base_link" tip_link="left_tool0"/>
  </group>
  
  <!-- Right arm with torso -->
  <group name="right_arm_torso">
    <joint name="torso_joint"/>
    <chain base_link="base_link" tip_link="right_tool0"/>
  </group>
</robot>
```

### Joint Collections

```xml
<robot name="redundant_arm">
  <!-- Plan using specific joints only -->
  <group name="arm_cartesian">
    <joint name="x_prismatic"/>
    <joint name="y_prismatic"/>
    <joint name="z_prismatic"/>
    <joint name="roll_revolute"/>
    <joint name="pitch_revolute"/>
    <joint name="yaw_revolute"/>
  </group>
  
  <!-- Redundant arm with all joints -->
  <group name="arm_full">
    <joint name="joint_1"/>
    <joint name="joint_2"/>
    <joint name="joint_3"/>
    <joint name="joint_4"/>
    <joint name="joint_5"/>
    <joint name="joint_6"/>
    <joint name="joint_7"/>
  </group>
  
  <!-- Subset for fast planning -->
  <group name="arm_fast">
    <joint name="joint_1"/>
    <joint name="joint_2"/>
    <joint name="joint_3"/>
  </group>
</robot>
```

## Collision Matrix

### Complete Collision Matrix Template

```xml
<robot name="my_robot">
  <!-- Adjacent links - always disable -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
  
  <!-- Non-adjacent but never in collision -->
  <disable_collisions link1="base_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="wrist_3_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="tool0" reason="Never"/>
  
  <!-- Links that can collide but are far apart -->
  <disable_collisions link1="shoulder_link" link2="wrist_3_link" reason="User"/>
  <disable_collisions link1="shoulder_link" link2="tool0" reason="User"/>
  
  <!-- Gripper finger collisions -->
  <disable_collisions link1="finger_1" link2="finger_2" reason="Adjacent"/>
  <disable_collisions link1="finger_1" link2="gripper_base" reason="Adjacent"/>
  <disable_collisions link1="finger_2" link2="gripper_base" reason="Adjacent"/>
</robot>
```

### Sensor Collision Exclusions

```xml
<robot name="robot_with_sensors">
  <!-- Camera never collides with anything -->
  <disable_collisions link1="camera_link" link2="base_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="shoulder_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="upper_arm_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="forearm_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="wrist_1_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="wrist_2_link" reason="Sensor"/>
  <disable_collisions link1="camera_link" link2="wrist_3_link" reason="Sensor"/>
  
  <!-- Cable management -->
  <disable_collisions link1="cable_carrier" link2="upper_arm_link" reason="User"/>
  <disable_collisions link1="cable_carrier" link2="forearm_link" reason="User"/>
  
  <!-- Tool attachments -->
  <disable_collisions link1="tool_mount" link2="wrist_3_link" reason="Adjacent"/>
  <disable_collisions link1="tool_mount" link2="tool0" reason="Adjacent"/>
</robot>
```

## State Configuration

### Named Configurations

```xml
<robot name="industrial_robot">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <!-- Standard poses -->
  <group_state name="all_zero" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="0"/>
    <joint name="elbow_joint" value="0"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <group_state name="ready" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-0.785"/>
    <joint name="elbow_joint" value="0.785"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <group_state name="work_position" group="manipulator">
    <joint name="shoulder_pan_joint" value="0.785"/>
    <joint name="shoulder_lift_joint" value="-1.57"/>
    <joint name="elbow_joint" value="0.785"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <!-- Safe transport pose -->
  <group_state name="transport" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="0"/>
    <joint name="elbow_joint" value="0"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <!-- Inspection pose (camera looking down) -->
  <group_state name="inspection" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.57"/>
    <joint name="elbow_joint" value="1.57"/>
    <joint name="wrist_1_joint" value="-1.57"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
</robot>
```

### Multi-Group States

```xml
<robot name="dual_arm">
  <group name="left_arm">
    <chain base_link="base_link" tip_link="left_tool"/>
  </group>
  
  <group name="right_arm">
    <chain base_link="base_link" tip_link="right_tool"/>
  </group>
  
  <!-- Coordinated dual-arm state -->
  <group_state name="dual_home" group="left_arm">
    <joint name="left_joint_1" value="0"/>
    <joint name="left_joint_2" value="-1.57"/>
    <joint name="left_joint_3" value="1.57"/>
    <joint name="left_joint_4" value="0"/>
    <joint name="left_joint_5" value="0"/>
    <joint name="left_joint_6" value="0"/>
  </group_state>
  
  <group_state name="dual_home" group="right_arm">
    <joint name="right_joint_1" value="0"/>
    <joint name="right_joint_2" value="-1.57"/>
    <joint name="right_joint_3" value="1.57"/>
    <joint name="right_joint_4" value="0"/>
    <joint name="right_joint_5" value="0"/>
    <joint name="right_joint_6" value="0"/>
  </group_state>
</robot>
```

## Complete Example

```xml
<?xml version="1.0"?>
<robot name="complete_robot">
  <!-- Groups -->
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <group name="gripper">
    <joint name="finger_joint"/>
  </group>
  
  <!-- End effector -->
  <end_effector name="tool0_ee" group="gripper" 
                parent_link="tool0" parent_group="manipulator"/>
  
  <!-- Virtual joint -->
  <virtual_joint name="fixed_base" type="fixed" 
                 parent_frame="world" child_link="base_link"/>
  
  <!-- States -->
  <group_state name="home" group="manipulator">
    <joint name="shoulder_pan_joint" value="0"/>
    <joint name="shoulder_lift_joint" value="-1.5708"/>
    <joint name="elbow_joint" value="1.5708"/>
    <joint name="wrist_1_joint" value="0"/>
    <joint name="wrist_2_joint" value="0"/>
    <joint name="wrist_3_joint" value="0"/>
  </group_state>
  
  <group_state name="open" group="gripper">
    <joint name="finger_joint" value="0.04"/>
  </group_state>
  
  <group_state name="close" group="gripper">
    <joint name="finger_joint" value="0.0"/>
  </group_state>
  
  <!-- Collision matrix -->
  <disable_collisions link1="base_link" link2="shoulder_link" reason="Adjacent"/>
  <disable_collisions link1="shoulder_link" link2="upper_arm_link" reason="Adjacent"/>
  <disable_collisions link1="upper_arm_link" link2="forearm_link" reason="Adjacent"/>
  <disable_collisions link1="forearm_link" link2="wrist_1_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_1_link" link2="wrist_2_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_2_link" link2="wrist_3_link" reason="Adjacent"/>
  <disable_collisions link1="wrist_3_link" link2="tool0" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="upper_arm_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="forearm_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="wrist_3_link" reason="Never"/>
  <disable_collisions link1="base_link" link2="tool0" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="forearm_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="wrist_3_link" reason="Never"/>
  <disable_collisions link1="shoulder_link" link2="tool0" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_1_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_2_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="wrist_3_link" reason="Never"/>
  <disable_collisions link1="upper_arm_link" link2="tool0" reason="Never"/>
</robot>
```
