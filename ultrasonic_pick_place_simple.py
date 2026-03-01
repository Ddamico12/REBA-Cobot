#!/usr/bin/env python3
"""
ultrasonic_pick_place_simple.py

Simple state machine:
- When "start" sensor sees object -> move to WAIT pose
- When "middle" sensor sees object -> close gripper, go MID, go PLACE, open gripper
- Return HOME and re-arm for next object

Requires:
- MoveIt group: "manipulator"
- Robotiq action server: /command_robotiq_action
- Topics: /ultrasonic_distance_start, /ultrasonic_distance_middle
"""

from collections import deque
import rospy
from std_msgs.msg import Float32
import moveit_commander
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal


class Debounce:
    """Require N consecutive samples satisfying a predicate."""
    def __init__(self, n_required: int):
        self.buf = deque(maxlen=max(1, int(n_required)))

    def push(self, value: float):
        self.buf.append(value)

    def ready(self) -> bool:
        return len(self.buf) == self.buf.maxlen

    def all_below(self, threshold: float) -> bool:
        return self.ready() and all(v < threshold for v in self.buf)


class UR3ePickPlaceSimple:
    def __init__(self):
        # ---- parameters ----
        self.start_threshold = float(rospy.get_param("~start_threshold_cm", 10.0))
        self.middle_threshold = float(rospy.get_param("~middle_threshold_cm", 10.0))
        self.invalid_below = float(rospy.get_param("~invalid_below_cm", 1.0))

        self.start_debounce_n = int(rospy.get_param("~start_debounce_n", 2))
        self.middle_debounce_n = int(rospy.get_param("~middle_debounce_n", 2))

        self.grab_delay_s = float(rospy.get_param("~grab_delay_s", 1.0))

        self.wait_joints = rospy.get_param("~wait_joints", [0.996646, -0.043245, 0.232846, -0.71984, -0.71352, 0.412095])
        self.mid_joints = rospy.get_param("~mid_joints", [-0.116615, -1.061636, 0.232702, -0.454744, -0.058025, 1.035356])
        self.place_joints = rospy.get_param("~place_joints", [-0.238117, -0.639414, 0.461706, -1.682276, -1.491387, -0.268583])

        self.move_vel = float(rospy.get_param("~velocity_scale", 0.85))
        self.move_acc = float(rospy.get_param("~accel_scale", 0.85))

        # ---- internal state ----
        self.seen_start = False
        self.busy = False

        self.start_db = Debounce(self.start_debounce_n)
        self.middle_db = Debounce(self.middle_debounce_n)

        # ---- MoveIt ----
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(self.move_vel)
        self.group.set_max_acceleration_scaling_factor(self.move_acc)

        # ---- Gripper action client ----
        self.gripper = actionlib.SimpleActionClient("/command_robotiq_action", CommandRobotiqGripperAction)
        rospy.loginfo("[simple] Waiting for gripper action server...")
        self.gripper.wait_for_server()

        # ---- startup posture ----
        rospy.loginfo("[simple] Moving to 'home' at startup...")
        self.group.set_named_target("home")
        self.group.go(wait=True)
        self.group.stop()

        rospy.loginfo("[simple] Opening gripper at startup...")
        self.set_gripper(open_ratio=1.0)

        # ---- subscribers ----
        rospy.Subscriber("/ultrasonic_distance_start", Float32, self.on_start)
        rospy.Subscriber("/ultrasonic_distance_middle", Float32, self.on_middle)

        rospy.loginfo("[simple] Ready. Waiting for object detection...")

    def set_gripper(self, open_ratio: float, speed: float = 255.0, force: float = 75):
        open_ratio = max(0.0, min(1.0, float(open_ratio)))
        goal = CommandRobotiqGripperGoal()
        goal.position = open_ratio * 255.0
        goal.speed = float(speed)
        goal.force = float(force)
        self.gripper.send_goal(goal)
        self.gripper.wait_for_result()

    def go_joints(self, joints):
        self.group.go(list(joints), wait=True)
        self.group.stop()

    def on_start(self, msg: Float32):
        d = float(msg.data)
        if d <= self.invalid_below:
            return
        self.start_db.push(d)

        if not self.busy and not self.seen_start and self.start_db.all_below(self.start_threshold):
            self.seen_start = True
            rospy.loginfo("[simple] Start sensor triggered -> moving to WAIT pose")
            self.go_joints(self.wait_joints)

    def on_middle(self, msg: Float32):
        d = float(msg.data)
        if d <= self.invalid_below:
            return
        self.middle_db.push(d)

        if self.busy or not self.seen_start:
            return

        if self.middle_db.all_below(self.middle_threshold):
            self.busy = True
            try:
                rospy.loginfo("[simple] Middle sensor triggered -> grabbing")
                rospy.sleep(self.grab_delay_s)
                self.set_gripper(open_ratio=0.0)

                rospy.loginfo("[simple] Moving MID -> PLACE")
                self.go_joints(self.mid_joints)
                self.go_joints(self.place_joints)

                rospy.loginfo("[simple] Releasing")
                self.set_gripper(open_ratio=1.0)

                rospy.loginfo("[simple] Returning HOME")
                self.group.set_named_target("home")
                self.group.go(wait=True)
                self.group.stop()

            finally:
                # reset for next cycle
                self.seen_start = False
                self.busy = False
                self.start_db.buf.clear()
                self.middle_db.buf.clear()
                rospy.loginfo("[simple] Cycle complete. Re-armed.")


def main():
    rospy.init_node("ultrasonic_pick_place_simple", anonymous=False)
    UR3ePickPlaceSimple()
    rospy.spin()


if __name__ == "__main__":
    main()