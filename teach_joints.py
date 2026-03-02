#!/usr/bin/env python3
"""
teach_joints.py

Interactive joint teaching script for conveyor_handoff_place.py.

Walks through 4 positions: move the robot by hand (or jog it),
press Enter to record each set of joint angles, then exports
a YAML file that can be loaded via rosparam.

Usage
-----
rosrun ur3e_moveit_config teach_joints.py

After teaching
--------------
rosparam load conveyor_joints.yaml
rosrun ur3e_moveit_config conveyor_handoff_place.py
"""

import os
import sys
import yaml
import rospy
import moveit_commander

# Positions to teach, in order.
# Each entry: (YAML key, human-readable prompt)
POSITIONS = [
    ("wait_pick_joints", "WAIT/PICK position"),
    ("handoff_joints",   "HANDOFF position"),
    ("mid_joints",       "MID position"),
    ("place_joints",     "PLACE position"),
]


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("teach_joints", anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")
    rospy.sleep(1.0)  # let MoveIt connect

    print("\n=== Joint Teaching for conveyor_handoff_place ===\n")
    print("For each position, move the robot to the desired pose,")
    print("then press Enter to record the joint values.\n")

    recorded = {}

    for key, label in POSITIONS:
        input(f"  -> Move the robot to {label}, then press Enter...")
        joints = group.get_current_joint_values()
        rounded = [round(float(j), 6) for j in joints]
        recorded[key] = rounded
        print(f"     Recorded {key}: {rounded}\n")

    # Build YAML structure matching the node's private param namespace
    data = {"conveyor_handoff_place": recorded}

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "conveyor_joints.yaml")
    with open(out_path, "w") as f:
        yaml.dump(data, f, default_flow_style=None)

    print("=" * 50)
    print(f"Saved to: {out_path}\n")
    print("To use these joints:\n")
    print(f"  rosparam load {out_path}")
    print("  rosrun ur3e_moveit_config conveyor_handoff_place.py\n")

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
