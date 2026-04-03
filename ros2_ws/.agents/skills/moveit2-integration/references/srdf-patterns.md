# SRDF Configuration Patterns

Advanced SRDF (Semantic Robot Description Format) configurations for complex MoveIt2 setups.

## Table of Contents

1. [Basic Structure](#basic-structure)
2. [Planning Groups](#planning-groups)
3. [End Effectors](#end-effectors)
4. [Self-Collision Matrix](#self-collision-matrix)
5. [Named Poses (Group States)](#named-poses-group-states)
6. [Multi-Arm Configuration](#multi-arm-configuration)
7. [Mobile Manipulator](#mobile-manipulator)
8. [Complex End Effector](#complex-end-effector)

---

## Basic Structure

```xml
<?xml version="1.0" encoding="UTF-8"?>
<!--
  SRDF for [Robot Name]
  Generated: [Date]
  
  Description:
    - 6-DOF manipulator
    - Parallel jaw gripper
    - Fixed base
-->
<robot name="robot_name">
  
  <!-- Virtual joint connects robot to world -->
  <virtual_joint name="virtual_joint" type="fixed" 
                 parent_frame="world" child_link="base_link"/>
  
  <!-- Planning groups -->
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <!-- End effectors -->
  <end_effector name="gripper" group="gripper" 
                parent_link="tool0" parent_group="arm"/>
  
  <!-- Named poses -->
  <group_state name="home" group="arm">
    <joint name="joint1" value="0"/>
    <joint name="joint2" value="0"/>
    <!-- ... -->
  </group_state>
  
  <!-- Self-collision disables -->
  <disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
  
</robot>
```

---

## Planning Groups

### Serial Chain (Most Common)

```xml
<group name="arm">
  <chain base_link="base_link" tip_link="tool0"/>
</group>
```

### Joint Collection

```xml
<group name="gripper">
  <joint name="finger_joint1"/>
  <joint name="finger_joint2"/>
  <joint name="gripper_palm_joint"/>
</group>
```

### Link Collection

```xml
<group name="hand">
  <link name="palm"/>
  <link name="finger_left"/>
  <link name="finger_right"/>
</group>
```

### Composite Group (Subgroups)

```xml
<group name="arm_and_gripper">
  <group name="arm"/>
  <group name="gripper"/>
</group>
```

### Redundant Groups (for compatibility)

```xml
<!-- Primary name -->
<group name="manipulator">
  <chain base_link="base_link" tip_link="tool0"/>
</group>

<!-- Alias for compatibility -->
<group name="arm">
  <chain base_link="base_link" tip_link="tool0"/>
</group>
```

---

## End Effectors

### Simple Gripper

```xml
<end_effector name="gripper" group="gripper" 
              parent_link="tool0" parent_group="arm"/>
```

### Multiple End Effectors

```xml
<!-- Left arm gripper -->
<end_effector name="left_gripper" group="left_gripper" 
              parent_link="left_tool0" parent_group="left_arm"/>

<!-- Right arm gripper -->
<end_effector name="right_gripper" group="right_gripper" 
              parent_link="right_tool0" parent_group="right_arm"/>

<!-- Suction cup on same arm (alternative) -->
<end_effector name="suction_tool" group="suction" 
              parent_link="arm_link6" parent_group="arm"/>
```

### End Effector with No Parent Group

```xml
<!-- Standalone end effector (for grasp planning only) -->
<end_effector name="detached_gripper" group="gripper" 
              parent_link="world" parent_group=""/>
```

---

## Self-Collision Matrix

### Disable Patterns

```xml
<!-- Adjacent links (always touch) -->
<disable_collisions link1="base_link" link2="link1" reason="Adjacent"/>
<disable_collisions link1="link1" link2="link2" reason="Adjacent"/>
<disable_collisions link1="link2" link2="link3" reason="Adjacent"/>
<disable_collisions link1="link3" link2="link4" reason="Adjacent"/>
<disable_collisions link1="link4" link2="link5" reason="Adjacent"/>
<disable_collisions link1="link5" link2="link6" reason="Adjacent"/>
<disable_collisions link1="link6" link2="tool0" reason="Adjacent"/>

<!-- Links that never collide (far apart) -->
<disable_collisions link1="base_link" link2="link3" reason="Never"/>
<disable_collisions link1="base_link" link2="link4" reason="Never"/>
<disable_collisions link1="base_link" link2="link5" reason="Never"/>
<disable_collisions link1="base_link" link2="link6" reason="Never"/>
<disable_collisions link1="base_link" link2="tool0" reason="Never"/>
<disable_collisions link1="link1" link2="link4" reason="Never"/>
<disable_collisions link1="link1" link2="link5" reason="Never"/>
<disable_collisions link1="link1" link2="link6" reason="Never"/>
<disable_collisions link1="link1" link2="tool0" reason="Never"/>
<disable_collisions link1="link2" link2="link5" reason="Never"/>
<disable_collisions link1="link2" link2="link6" reason="Never"/>
<disable_collisions link1="link2" link2="tool0" reason="Never"/>
<disable_collisions link1="link3" link2="tool0" reason="Never"/>

<!-- Gripper internal collisions -->
<disable_collisions link1="gripper_base" link2="finger_left" reason="Adjacent"/>
<disable_collisions link1="gripper_base" link2="finger_right" reason="Adjacent"/>
<disable_collisions link1="finger_left" link2="finger_right" reason="Never"/>
```

### Reason Categories

| Reason | Description |
|--------|-------------|
| `Adjacent` | Links are adjacent in kinematic chain |
| `Never` | Links can never collide (too far apart) |
| `Default` | Links in default position collision |
| `Always` | Links always in collision (design issue) |
| `User` | Manually disabled by user |

---

## Named Poses (Group States)

### Home and Ready Positions

```xml
<!-- Home position (all zeros) -->
<group_state name="home" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="0"/>
  <joint name="joint3" value="0"/>
  <joint name="joint4" value="0"/>
  <joint name="joint5" value="0"/>
  <joint name="joint6" value="0"/>
</group_state>

<!-- Ready position (typical working pose) -->
<group_state name="ready" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="-0.785"/>
  <joint name="joint3" value="0"/>
  <joint name="joint4" value="-2.356"/>
  <joint name="joint5" value="0"/>
  <joint name="joint6" value="1.571"/>
</group_state>

<!-- Tucked position (for transport) -->
<group_state name="tucked" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="-1.57"/>
  <joint name="joint3" value="3.14"/>
  <joint name="joint4" value="-2.8"/>
  <joint name="joint5" value="0"/>
  <joint name="joint6" value="0"/>
</group_state>

<!-- Extended position (full reach) -->
<group_state name="extended" group="arm">
  <joint name="joint1" value="0"/>
  <joint name="joint2" value="0"/>
  <joint name="joint3" value="0"/>
  <joint name="joint4" value="0"/>
  <joint name="joint5" value="1.57"/>
  <joint name="joint6" value="0"/>
</group_state>
```

### Gripper States

```xml
<group_state name="open" group="gripper">
  <joint name="finger_joint" value="0.04"/>
</group_state>

<group_state name="close" group="gripper">
  <joint name="finger_joint" value="0"/>
</group_state>

<group_state name="half_open" group="gripper">
  <joint name="finger_joint" value="0.02"/>
</group_state>
```

---

## Multi-Arm Configuration

```xml
<robot name="dual_arm_robot">
  <!-- Virtual joint -->
  <virtual_joint name="virtual_joint" type="fixed" 
                 parent_frame="world" child_link="base_link"/>
  
  <!-- Left arm -->
  <group name="left_arm">
    <chain base_link="base_link" tip_link="left_tool0"/>
  </group>
  
  <!-- Right arm -->
  <group name="right_arm">
    <chain base_link="base_link" tip_link="right_tool0"/>
  </group>
  
  <!-- Both arms (for coordinated motion) -->
  <group name="both_arms">
    <group name="left_arm"/>
    <group name="right_arm"/>
  </group>
  
  <!-- End effectors -->
  <end_effector name="left_hand" group="left_gripper" 
                parent_link="left_tool0" parent_group="left_arm"/>
  <end_effector name="right_hand" group="right_gripper" 
                parent_link="right_tool0" parent_group="right_arm"/>
  
  <!-- Poses for each arm -->
  <group_state name="home" group="left_arm">
    <joint name="left_joint1" value="0"/>
    <!-- ... -->
  </group_state>
  
  <group_state name="home" group="right_arm">
    <joint name="right_joint1" value="0"/>
    <!-- ... -->
  </group_state>
  
  <!-- Cross-arm collision disables (if arms can't collide) -->
  <disable_collisions link1="left_link1" link2="right_link1" reason="Never"/>
  <disable_collisions link1="left_link2" link2="right_link2" reason="Never"/>
  <!-- ... -->
  
</robot>
```

---

## Mobile Manipulator

```xml
<robot name="mobile_manipulator">
  <!-- Virtual planar joint for mobile base -->
  <virtual_joint name="virtual_joint" type="planar" 
                 parent_frame="odom" child_link="base_footprint"/>
  
  <!-- Base group (mobile platform) -->
  <group name="base">
    <joint name="base_x_joint"/>
    <joint name="base_y_joint"/>
    <joint name="base_theta_joint"/>
  </group>
  
  <!-- Arm group -->
  <group name="arm">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  
  <!-- Whole body planning -->
  <group name="whole_body">
    <group name="base"/>
    <group name="arm"/>
  </group>
  
  <!-- End effector -->
  <end_effector name="gripper" group="gripper" 
                parent_link="tool0" parent_group="arm"/>
  
  <!-- Named poses -->
  <group_state name="home" group="arm">
    <joint name="joint1" value="0"/>
    <!-- ... -->
  </group_state>
  
  <!-- Base-specific poses -->
  <group_state name="centered" group="base">
    <joint name="base_x_joint" value="0"/>
    <joint name="base_y_joint" value="0"/>
    <joint name="base_theta_joint" value="0"/>
  </group_state>
  
</robot>
```

---

## Complex End Effector

### Multi-Finger Hand

```xml
<robot name="humanoid_hand_robot">
  
  <!-- Main arm -->
  <group name="arm">
    <chain base_link="base_link" tip_link="wrist"/>
  </group>
  
  <!-- Individual fingers -->
  <group name="thumb">
    <chain base_link="wrist" tip_link="thumb_tip"/>
  </group>
  
  <group name="index_finger">
    <chain base_link="wrist" tip_link="index_tip"/>
  </group>
  
  <group name="middle_finger">
    <chain base_link="wrist" tip_link="middle_tip"/>
  </group>
  
  <group name="ring_finger">
    <chain base_link="wrist" tip_link="ring_tip"/>
  </group>
  
  <group name="pinky">
    <chain base_link="wrist" tip_link="pinky_tip"/>
  </group>
  
  <!-- Combined hand group -->
  <group name="hand">
    <group name="thumb"/>
    <group name="index_finger"/>
    <group name="middle_finger"/>
    <group name="ring_finger"/>
    <group name="pinky"/>
  </group>
  
  <!-- Arm + hand -->
  <group name="arm_and_hand">
    <group name="arm"/>
    <group name="hand"/>
  </group>
  
  <!-- Poses for fingers -->
  <group_state name="open" group="hand">
    <joint name="thumb_joint1" value="0"/>
    <joint name="thumb_joint2" value="0"/>
    <joint name="index_joint1" value="0"/>
    <joint name="index_joint2" value="0"/>
    <!-- ... -->
  </group_state>
  
  <group_state name="fist" group="hand">
    <joint name="thumb_joint1" value="0.5"/>
    <joint name="thumb_joint2" value="0.8"/>
    <joint name="index_joint1" value="1.57"/>
    <joint name="index_joint2" value="1.57"/>
    <!-- ... -->
  </group_state>
  
  <group_state name="pinch" group="hand">
    <joint name="thumb_joint1" value="0.3"/>
    <joint name="thumb_joint2" value="0.5"/>
    <joint name="index_joint1" value="0.3"/>
    <joint name="index_joint2" value="0.5"/>
    <!-- Other fingers curled -->
    <joint name="middle_joint1" value="1.57"/>
    <!-- ... -->
  </group_state>
  
</robot>
```

---

## Passive Joints

```xml
<!-- Passive casters on mobile base -->
<passive_joint name="caster_front_left_joint"/>
<passive_joint name="caster_front_right_joint"/>
<passive_joint name="caster_back_left_joint"/>
<passive_joint name="caster_back_right_joint"/>

<!-- Unactuated parallel mechanism joints -->
<passive_joint name="parallel_link_joint"/>
```

---

## SRDF Validation Checklist

- [ ] `virtual_joint` defined and connects to world frame
- [ ] All planning groups have valid joints/links
- [ ] End effectors reference existing groups and links
- [ ] No overlapping links between end effector and parent group
- [ ] Named poses specify all joints in the group
- [ ] Self-collision disables have valid reasons
- [ ] Joint names match URDF exactly
- [ ] Link names match URDF exactly
