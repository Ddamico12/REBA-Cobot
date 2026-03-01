#!/usr/bin/env python3
"""
ultrasonic_pick_place_advanced.py

Advanced version:
- START trigger -> go to WAIT
- MIDDLE trigger -> close gripper, go through MID[index] -> PLACE[index], open gripper
- Return HOME each cycle
- Optional "push routine" after a specified index (e.g., after 3rd pick)

This keeps your multi-placement idea but is fully rewritten.
"""

from collections import deque
import rospy
from std_msgs.msg import Float32
import moveit_commander
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal


class ConsecutiveTrigger:
    def __init__(self, n: int):
        self.buf = deque(maxlen=max(1, int(n)))

    def update(self, value: float):
        self.buf.append(value)

    def fired_below(self, threshold: float) -> bool:
        return len(self.buf) == self.buf.maxlen and all(v < threshold for v in self.buf)

    def reset(self):
        self.buf.clear()


class UR3ePickPlaceAdvanced:
    def __init__(self):
        # thresholds
        self.start_threshold = float(rospy.get_param("~start_threshold_cm", 15.0))
        self.middle_threshold = float(rospy.get_param("~middle_threshold_cm", 15.0))
        self.invalid_below = float(rospy.get_param("~invalid_below_cm", 1.0))

        # debouncing
        self.start_n = int(rospy.get_param("~start_debounce_n", 2))
        self.middle_n = int(rospy.get_param("~middle_debounce_n", 2))
        self.start_trig = ConsecutiveTrigger(self.start_n)
        self.middle_trig = ConsecutiveTrigger(self.middle_n)

        # timing
        self.grab_delay_s = float(rospy.get_param("~grab_delay_s", 0.7))

        # motion tuning
        self.vel = float(rospy.get_param("~velocity_scale", 0.85))
        self.acc = float(rospy.get_param("~accel_scale", 0.85))

        # poses
        self.wait_joints = rospy.get_param("~wait_joints", [0.961, -0.867, 0.603, -1.309, -1.567, 0.954])

        # lists: MID and PLACE must be same length
        self.mid_list = rospy.get_param("~mid_list", [
            [1.364, -1.341, -1.333, -0.458, 0.307, 0.000],
            [0.465, -2.557, 1.363, -1.942, 1.207, 0.000],
            [-0.568, -2.632, 0.933, -1.438, 2.241, 0.000],
        ])

        self.place_list = rospy.get_param("~place_list", [
            [0.800, -2.066, -0.482, -0.589, 0.872, 0.000],
            [0.283, -2.571, 0.661, -1.227, 1.390, 0.000],
            [-0.339, -2.412, 0.179, -0.903, 2.012, 0.000],
        ])

        # push routine (optional)
        self.enable_push = bool(rospy.get_param("~enable_push", False))
        self.push_after_index = int(rospy.get_param("~push_after_index", 2))
        self.push_list = rospy.get_param("~push_list", [
            [0.792, -2.432, 0.250, -0.954, 0.881, 0.000],
            [0.277, -2.546, 0.568, -1.160, 1.396, 0.000],
            [-0.351, -2.157, -0.349, -0.631, 2.023, 0.000],
        ])

        # runtime state
        self.armed = False
        self.busy = False
        self.idx = 0

        # init moveit
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(self.vel)
        self.group.set_max_acceleration_scaling_factor(self.acc)

        # gripper action
        self.gripper = actionlib.SimpleActionClient("/command_robotiq_action", CommandRobotiqGripperAction)
        rospy.loginfo("[advanced] Waiting for gripper action server...")
        self.gripper.wait_for_server()

        # startup
        rospy.loginfo("[advanced] Moving to 'home' at startup...")
        self.group.set_named_target("home")
        self.group.go(wait=True)
        self.group.stop()
        rospy.loginfo("[advanced] Opening gripper at startup...")
        self.set_gripper(1.0, force=50.0)

        # topics
        rospy.Subscriber("/ultrasonic_distance_start", Float32, self.cb_start)
        rospy.Subscriber("/ultrasonic_distance_middle", Float32, self.cb_middle)

        rospy.loginfo("[advanced] Ready.")

    def set_gripper(self, open_ratio: float, speed: float = 255.0, force: float = 10.0):
        open_ratio = max(0.0, min(1.0, float(open_ratio)))
        goal = CommandRobotiqGripperGoal()
        goal.position = open_ratio * 255.0
        goal.speed = float(speed)
        goal.force = float(force)
        self.gripper.send_goal(goal)
        self.gripper.wait_for_result()

    def go(self, joints):
        self.group.go(list(joints), wait=True)
        self.group.stop()

    def go_home(self):
        self.group.set_named_target("home")
        self.group.go(wait=True)
        self.group.stop()

    def cb_start(self, msg: Float32):
        d = float(msg.data)
        if d <= self.invalid_below:
            return
        self.start_trig.update(d)

        if (not self.busy) and (not self.armed) and self.start_trig.fired_below(self.start_threshold):
            self.armed = True
            rospy.loginfo("[advanced] START fired -> going to WAIT")
            self.go(self.wait_joints)

    def cb_middle(self, msg: Float32):
        d = float(msg.data)
        if d <= self.invalid_below:
            return
        self.middle_trig.update(d)

        if self.busy or not self.armed:
            return

        if not self.middle_trig.fired_below(self.middle_threshold):
            return

        # do one full cycle
        self.busy = True
        try:
            if len(self.mid_list) != len(self.place_list):
                rospy.logerr("[advanced] mid_list and place_list length mismatch")
                return

            use_i = self.idx % len(self.place_list)

            rospy.loginfo(f"[advanced] MIDDLE fired -> grab + place index {use_i}")
            rospy.sleep(self.grab_delay_s)
            self.set_gripper(0.0)

            self.go(self.mid_list[use_i])
            self.go(self.place_list[use_i])

            rospy.sleep(0.2)
            self.set_gripper(1.0)

            self.go_home()

            # optional push routine
            if self.enable_push and use_i == self.push_after_index:
                rospy.loginfo("[advanced] Running push routine...")
                self.set_gripper(0.0)  # close while pushing (optional)
                for a in range(min(len(self.push_list), len(self.mid_list))):
                    self.go(self.mid_list[a])
                    self.go(self.push_list[a])
                    self.go(self.mid_list[a])
                self.set_gripper(1.0)
                self.go_home()

            self.idx += 1

        finally:
            self.armed = False
            self.busy = False
            self.start_trig.reset()
            self.middle_trig.reset()
            rospy.loginfo("[advanced] Cycle complete. Re-armed.")


def main():
    rospy.init_node("ultrasonic_pick_place_advanced", anonymous=False)
    UR3ePickPlaceAdvanced()
    rospy.spin()


if __name__ == "__main__":
    main()