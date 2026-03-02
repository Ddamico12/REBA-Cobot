#!/usr/bin/env python3
"""
teach_joints.py

Interactive joint + parameter teaching script for conveyor_handoff_place.py.

Adds two extra taught waypoints:
  - pre_handoff_joints: between wait_pick_joints and handoff_joints
  - post_place_joints: between place_joints and home

Workflow:
  1) Enter/tune parameters (Enter keeps defaults).
  2) Teach all positions by moving robot and pressing Enter.
  3) Exports conveyor_joints.yaml containing BOTH params + joint targets.

Usage:
  rosrun ur3e_moveit_config teach_joints.py

After teaching:
  rosparam load <path-to>/conveyor_joints.yaml
  rosrun ur3e_moveit_config conveyor_handoff_place.py
"""

import os
import sys
import yaml
import rospy
import moveit_commander

# Positions to teach, in order:
# Each entry: (YAML key, human-readable prompt)
POSITIONS = [
    ("wait_pick_joints",    "WAIT/PICK position"),
    ("pre_handoff_joints",  "PRE-HANDOFF mid position (between WAIT/PICK and HANDOFF)"),
    ("handoff_joints",      "HANDOFF position"),
    ("mid_joints",          "MID position (for large-box path between PICK and PLACE)"),
    ("place_joints",        "PLACE position"),
    ("post_place_joints",   "POST-PLACE mid position (between PLACE and HOME)"),
]


def _prompt_float(prompt: str, default: float, lo=None, hi=None) -> float:
    while True:
        s = input(f"{prompt} [{default}]: ").strip()
        if s == "":
            v = float(default)
        else:
            try:
                v = float(s)
            except ValueError:
                print("  -> Please enter a number.")
                continue

        if lo is not None and v < lo:
            print(f"  -> Must be >= {lo}.")
            continue
        if hi is not None and v > hi:
            print(f"  -> Must be <= {hi}.")
            continue
        return v


def _prompt_int(prompt: str, default: int, lo=None, hi=None) -> int:
    while True:
        s = input(f"{prompt} [{default}]: ").strip()
        if s == "":
            v = int(default)
        else:
            try:
                v = int(s)
            except ValueError:
                print("  -> Please enter an integer.")
                continue

        if lo is not None and v < lo:
            print(f"  -> Must be >= {lo}.")
            continue
        if hi is not None and v > hi:
            print(f"  -> Must be <= {hi}.")
            continue
        return v


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("teach_joints", anonymous=True)

    group = moveit_commander.MoveGroupCommander("manipulator")
    rospy.sleep(1.0)  # let MoveIt connect

    print("\n=== Joint + Parameter Teaching for conveyor_handoff_place ===\n")
    print("Step 1: Enter/tune parameters (press Enter to keep defaults).")
    print("Step 2: For each position, move the robot, then press Enter to record joints.\n")

    # ---------------------------
    # Parameter inputs
    # ---------------------------
    print("=== Parameters ===")
    taken_wait_s     = _prompt_float("Human 'take box' wait time (s)", 0.3, lo=0.0, hi=10.0)
    grab_delay_s     = _prompt_float("Delay after Sensor 2 before CLOSE (s)", 1.5, lo=0.0, hi=10.0)
    num_small_boxes  = _prompt_int("Number of small-box handoffs per cycle", 3, lo=1, hi=20)

    velocity_scale   = _prompt_float("MoveIt velocity scale (0-1)", 0.9, lo=0.01, hi=1.0)
    accel_scale      = _prompt_float("MoveIt accel scale (0-1)", 0.9, lo=0.01, hi=1.0)

    gripper_speed    = _prompt_float("Gripper speed (0-255)", 255.0, lo=0.0, hi=255.0)

    # Optional sensor params (you can keep defaults if you don't want to tune here)
    start_threshold  = _prompt_float("Sensor 1 threshold (cm)", 30.0, lo=0.0, hi=500.0)
    middle_threshold = _prompt_float("Sensor 2 threshold (cm)", 30.0, lo=0.0, hi=500.0)
    debounce_n       = _prompt_int("Debounce N (samples)", 2, lo=1, hi=20)
    sensor2_timeout  = _prompt_float("Sensor 2 timeout (s)", 5.0, lo=0.0, hi=60.0)

    print("\n=== Teaching Joints ===")
    print("For each position:")
    print("  - Move/jog the robot to the desired pose")
    print("  - Press Enter to record the current joint angles\n")

    recorded = {}

    for key, label in POSITIONS:
        input(f"  -> Move the robot to {label}, then press Enter...")
        joints = group.get_current_joint_values()
        rounded = [round(float(j), 6) for j in joints]
        recorded[key] = rounded
        print(f"     Recorded {key}: {rounded}\n")

    # ---------------------------
    # Build YAML structure
    # ---------------------------
    cfg = {
        # Timing / cycle
        "taken_wait_s": float(taken_wait_s),
        "grab_delay_s": float(grab_delay_s),
        "num_small_boxes": int(num_small_boxes),

        # Motion scaling
        "velocity_scale": float(velocity_scale),
        "accel_scale": float(accel_scale),

        # Gripper
        "gripper_speed": float(gripper_speed),

        # Sensor tuning (optional but helpful to store)
        "start_threshold_cm": float(start_threshold),
        "middle_threshold_cm": float(middle_threshold),
        "debounce_n": int(debounce_n),
        "sensor2_timeout_s": float(sensor2_timeout),
    }

    # Merge joint recordings
    cfg.update(recorded)

    # Match the node name/namespace used by conveyor_handoff_place.py
    data = {"conveyor_handoff_place": cfg}

    # Save next to this script by default
    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "conveyor_joints.yaml")

    with open(out_path, "w") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            default_flow_style=False
        )

    print("=" * 60)
    print(f"Saved YAML to:\n  {out_path}\n")
    print("To use it:")
    print(f"  rosparam load {out_path}")
    print("  rosrun ur3e_moveit_config conveyor_handoff_place.py\n")
    print("Verify params:")
    print("  rosparam get /conveyor_handoff_place/pre_handoff_joints")
    print("  rosparam get /conveyor_handoff_place/post_place_joints")
    print("=" * 60)

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()