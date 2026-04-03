#!/usr/bin/env python3
"""
Generate collision matrix SRDF entries from URDF.
Usage: python3 generate_collision_matrix.py <urdf_file>
"""

import sys
import xml.etree.ElementTree as ET
from itertools import combinations


def parse_urdf(urdf_file):
    """Parse URDF and extract links and joints"""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    links = [link.get('name') for link in root.findall('.//link')]
    
    # Find parent-child relationships through joints
    adjacencies = set()
    for joint in root.findall('.//joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            parent_name = parent.get('link')
            child_name = child.get('link')
            adjacencies.add((parent_name, child_name))
            adjacencies.add((child_name, parent_name))
    
    return links, adjacencies


def generate_disable_collisions(links, adjacencies):
    """Generate disable_collisions entries"""
    disabled = set()
    
    # Disable adjacent links
    for link1, link2 in adjacencies:
        if (link2, link1) not in disabled:
            disabled.add((link1, link2))
            print(f'  <disable_collisions link1="{link1}" link2="{link2}" reason="Adjacent"/>')
    
    # Disable collisions between links that are far apart (heuristic)
    # This is a simple heuristic - you may want to customize
    base_links = ['base_link', 'base', 'world']
    base = None
    for b in base_links:
        if b in links:
            base = b
            break
    
    if base:
        end_effector_links = [l for l in links if 'tool' in l or 'ee' in l or 'gripper' in l]
        for ee in end_effector_links:
            if (base, ee) not in disabled and (ee, base) not in disabled:
                # These are never in collision due to arm length
                disabled.add((base, ee))
                print(f'  <disable_collisions link1="{base}" link2="{ee}" reason="Never"/>')


def main():
    if len(sys.argv) < 2:
        print("Usage: generate_collision_matrix.py <urdf_file>")
        print("\nThis script generates SRDF disable_collisions entries.")
        print("You should review and customize the output for your robot.")
        sys.exit(1)
    
    urdf_file = sys.argv[1]
    
    try:
        links, adjacencies = parse_urdf(urdf_file)
    except Exception as e:
        print(f"Error parsing URDF: {e}")
        sys.exit(1)
    
    print(f"<!-- Collision matrix generated from {urdf_file} -->")
    print(f"<!-- Found {len(links)} links, {len(adjacencies)//2} joints -->")
    print("<!-- Review and customize for your robot! -->")
    print()
    
    generate_disable_collisions(links, adjacencies)


if __name__ == '__main__':
    main()
