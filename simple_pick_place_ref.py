#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UR3e sequence (ROS1 Noetic + MoveIt) using your RobotController:
HOME -> pos1 -> pos2 (CLOSE) -> pos3 -> pos4 (OPEN) -> pos5 -> HOME

Put this file in the SAME folder as your reference controller file.
Assumption: reference file is named `robot_controller.py` and contains RobotController.
"""

import rospy
from robot_controller import RobotController  # change if your controller filename differs


def _as_6(name, joints):
    if not isinstance(joints, (list, tuple)) or len(joints) != 6:
        raise ValueError(f"{name} must be a list/tuple of 6 values. Got: {joints}")
    return [float(x) for x in joints]


def main():
    rospy.init_node("move_5_positions_gripper", anonymous=True)

    rc = RobotController(move_group_name=rospy.get_param("~move_group", "manipulator"))

    # ---- Joint Positions ----
    pos1 = _as_6("pos1", [-1.298714, -2.362564, -1.023759, -3.249811, -1.296968, 0.121822])
    pos2 = _as_6("pos2", [-1.299568, -2.547044, -0.966051, -3.273039, -1.320683, 0.137601])
    pos3 = _as_6("pos3", [-0.020269, -2.15761,  -0.179658, -1.355088, -1.619437, 0.016956])
    pos4 = _as_6("pos4", [-0.004358, -1.09856,  -0.162894, -2.111252, -1.554373, 0.053511])
    pos5 = _as_6("pos5", [0.025707, -1.210326, -0.136873, -3.367004, -1.579377, 0.022707])

    # ---- Gripper Settings (per your RobotController docstring: close=0.0, open=1.0) ----
    use_gripper = rospy.get_param("~use_gripper", True)
    close_frac = float(rospy.get_param("~close_frac", 0.0))
    open_frac = float(rospy.get_param("~open_frac", 1.0))
    grip_speed = float(rospy.get_param("~grip_speed", 0.1))
    grip_force = float(rospy.get_param("~grip_force", 10.0))

    # Optional dwell (seconds) after gripper actions
    grip_pause = float(rospy.get_param("~grip_pause", 0.3))

    rospy.loginfo("=== Sequence start: HOME -> 1 -> 2(close) -> 3 -> 4(open) -> 5 -> HOME ===")

    # HOME (start)
    if not rc.go_home():
        rospy.logerr("Failed to go home at start.")
        return

    # Position 1
    rospy.loginfo("Moving to position 1...")
    if not rc.move_to_joints(pos1):
        rospy.logerr("Failed to reach position 1.")
        return

    # Position 2
    rospy.loginfo("Moving to position 2...")
    if not rc.move_to_joints(pos2):
        rospy.logerr("Failed to reach position 2.")
        return

    # Close gripper at position 2
    if use_gripper:
        rospy.loginfo("Closing gripper at position 2 (grab)...")
        rc.control_gripper(close_frac, speed=grip_speed, force=grip_force)
        rospy.sleep(grip_pause)

    # Position 3
    rospy.loginfo("Moving to position 3...")
    if not rc.move_to_joints(pos3):
        rospy.logerr("Failed to reach position 3.")
        return

    # Position 4
    rospy.loginfo("Moving to position 4...")
    if not rc.move_to_joints(pos4):
        rospy.logerr("Failed to reach position 4.")
        return

    # Open gripper at position 4
    if use_gripper:
        rospy.loginfo("Opening gripper at position 4 (release)...")
        rc.control_gripper(open_frac, speed=grip_speed, force=grip_force)
        rospy.sleep(grip_pause)

    # Position 5
    rospy.loginfo("Moving to position 5...")
    if not rc.move_to_joints(pos5):
        rospy.logerr("Failed to reach position 5.")
        return

    # HOME (end)
    rospy.loginfo("Returning home after position 5...")
    if not rc.go_home():
        rospy.logerr("Failed to return home.")
        return

    rospy.loginfo("=== Sequence complete ===")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Exception: {e}")