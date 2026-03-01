#!/usr/bin/env python3
"""
ultrasonic_handoff_bundle_sensor2_gated_taken_by_position.py

WHAT THIS DOES (runs until Ctrl-C)
----------------------------------
Conveyor order (right -> left): Box -> Sensor 1 -> Sensor 2 -> Robot

Hard requirements you gave:
1) Sensor 1 ONLY tells the robot to go to WAIT/PICK position.
2) Gripper CLOSE happens ONLY when Sensor 2 triggers.
3) Sensor 2 is only allowed to trigger after Sensor 1 (gated/armed).
4) Robot hands 3 single boxes to a human, then waits for a taped bundle,
   then moves bundle to shelf and releases, then returns HOME.
5) Robot must detect when the human TAKES the box from the gripper.

TAKEN detection (improved):
- Uses GRIPPER POSITION CHANGE from /command_robotiq_action/feedback
- When the human pulls the box out, the fingers open more -> position increases.
- We record baseline position at handoff and detect a stable increase >= taken_delta_pos.

Gripper position units (auto-detect):
- Some drivers use meters (0.0 closed ... ~0.085 open)
- Others use raw 0..255
We auto-detect via feedback.requested_position.

RUN
---
chmod +x ultrasonic_handoff_bundle_sensor2_gated_taken_by_position.py
source ~/main_workspace/catkin_ws/devel/setup.bash
rosrun ur3e_moveit_config ultrasonic_handoff_bundle_sensor2_gated_taken_by_position.py

Useful params to tune:
  _sensor1_threshold_cm:=30
  _sensor2_threshold_cm:=30
  _force_close_on_sensor2:=60
  _handoff_relax_open_ratio:=0.15
  _taken_delta_pos:=0.008
  _taken_stable_n:=8
  _sensor2_timeout_s:=5
"""

import threading
from collections import deque
import rospy
from std_msgs.msg import Float32

import moveit_commander
import actionlib
from robotiq_2f_gripper_msgs.msg import (
    CommandRobotiqGripperAction,
    CommandRobotiqGripperGoal,
    CommandRobotiqGripperActionFeedback,
)


# -------------------------- Utilities --------------------------


class ConsecutiveBelow:
    """Debounce helper: triggers when last N samples are all below threshold."""
    def __init__(self, n_required: int):
        self.buf = deque(maxlen=max(1, int(n_required)))

    def push(self, v: float):
        self.buf.append(float(v))

    def fired(self, threshold: float) -> bool:
        return len(self.buf) == self.buf.maxlen and all(x < threshold for x in self.buf)

    def reset(self):
        self.buf.clear()


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


# -------------------------- Main Node --------------------------


class UR3eHandoffBundle:
    def __init__(self):
        # ---------------- ROS topics ----------------
        self.topic_sensor1 = rospy.get_param("~topic_sensor1", "/ultrasonic_distance_start")    # Sensor 1
        self.topic_sensor2 = rospy.get_param("~topic_sensor2", "/ultrasonic_distance_middle")  # Sensor 2

        # ---------------- Ultrasonic parameters ----------------
        self.invalid_below_cm = float(rospy.get_param("~invalid_below_cm", 1.0))
        self.sensor1_threshold_cm = float(rospy.get_param("~sensor1_threshold_cm", 30.0))
        self.sensor2_threshold_cm = float(rospy.get_param("~sensor2_threshold_cm", 30.0))
        self.debounce_n = int(rospy.get_param("~debounce_n", 2))

        # Sensor 2 gating timeout: if Sensor 2 doesn't happen soon after Sensor 1, disarm and re-wait Sensor 1.
        self.sensor2_timeout_s = float(rospy.get_param("~sensor2_timeout_s", 5.0))

        # ---------------- Cycle parameters ----------------
        self.num_boxes = int(rospy.get_param("~num_boxes", 3))
        self.pause_after_move_s = float(rospy.get_param("~pause_after_move_s", 0.15))
        self.idle_between_cycles_s = float(rospy.get_param("~idle_between_cycles_s", 0.5))
        self.close_delay_after_sensor2_s = float(rospy.get_param("~close_delay_after_sensor2_s", 0.0))

        # ---------------- Motion scaling ----------------
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 0.20))
        self.accel_scale = float(rospy.get_param("~accel_scale", 0.20))

        # ---------------- Gripper command parameters ----------------
        self.gripper_speed = float(rospy.get_param("~gripper_speed", 1))

        # This is the ONLY force used for a CLOSE command (and CLOSE happens ONLY at Sensor 2).
        # Choose it to be "secure but easy to pull from the handoff".
        self.force_close_on_sensor2 = float(rospy.get_param("~force_close_on_sensor2", 60.0))

        # Open command force (doesn't clamp on object; fine as-is).
        self.force_open = float(rospy.get_param("~force_open", 40.0))

        # If gripper is meters mode, this is typical max opening width for Robotiq 2F-85.
        self.gripper_max_open_m = float(rospy.get_param("~gripper_max_open_m", 0.085))
        self._pos_mode = None  # "meters" or "raw255"

        # Handoff "relax": OPEN slightly at the handoff pose to make pulling easier (OPEN only, not close).
        self.handoff_relax_open_ratio = float(rospy.get_param("~handoff_relax_open_ratio", 0.15))

        # ---------------- TAKEN detection (position change) ----------------
        self.taken_timeout_s = float(rospy.get_param("~taken_timeout_s", 12.0))
        self.taken_stable_n = int(rospy.get_param("~taken_stable_n", 8))
        # meters-mode typical: 0.005 to 0.015
        # raw255 typical: 5 to 25
        self.taken_delta_pos = float(rospy.get_param("~taken_delta_pos", 0.008))

        # ---------------- Joint targets (your updated positions) ----------------
        self.wait_pick_joints = rospy.get_param(
            "~wait_pick_joints",
            [0.996646, -0.043245, 0.232846, -0.71984, -0.71352, 0.412095],
        )
        self.handoff_joints = rospy.get_param(
            "~handoff_joints",
            [1.676746, -0.200665, 0.384843, -0.100697, 1.627355, -0.021775],
        )
        self.mid_bundle_joints = rospy.get_param(
            "~mid_bundle_joints",
            [-0.264171, -1.512796, 0.204514, 0.499847, 0.933737, 0.458285],
        )
        self.place_bundle_joints = rospy.get_param(
            "~place_bundle_joints",
            [-0.125007, -0.812937, 0.743014, -2.023574, -1.532625, -0.016113],
        )

        # ---------------- Debouncers ----------------
        self.db1 = ConsecutiveBelow(self.debounce_n)
        self.db2 = ConsecutiveBelow(self.debounce_n)

        # ---------------- Gating state ----------------
        self.sensor2_armed = False
        self.sensor2_arm_time = None

        # ---------------- MoveIt ----------------
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(self.velocity_scale)
        self.group.set_max_acceleration_scaling_factor(self.accel_scale)

        # ---------------- Robotiq Action ----------------
        self.gripper = actionlib.SimpleActionClient("/command_robotiq_action", CommandRobotiqGripperAction)
        rospy.loginfo("[handoff] Waiting for /command_robotiq_action...")
        self.gripper.wait_for_server()
        rospy.loginfo("[handoff] Connected to gripper action server.")

        # ---------------- Gripper feedback ----------------
        self._fb_lock = threading.Lock()
        self._fb = None
        rospy.Subscriber(
            "/command_robotiq_action/feedback",
            CommandRobotiqGripperActionFeedback,
            self._cb_gripper_feedback,
            queue_size=10,
        )

        # ---------------- Ultrasonic subscribers ----------------
        rospy.Subscriber(self.topic_sensor1, Float32, self._cb_sensor1, queue_size=50)
        rospy.Subscriber(self.topic_sensor2, Float32, self._cb_sensor2, queue_size=50)

        rospy.loginfo("[handoff] Node initialized (Sensor2 gated by Sensor1, TAKEN by position).")

    # ---------------- callbacks ----------------
    def _cb_gripper_feedback(self, msg: CommandRobotiqGripperActionFeedback):
        with self._fb_lock:
            self._fb = msg.feedback

    def _cb_sensor1(self, msg: Float32):
        d = float(msg.data)
        if d > self.invalid_below_cm:
            self.db1.push(d)

    def _cb_sensor2(self, msg: Float32):
        d = float(msg.data)
        if d > self.invalid_below_cm:
            self.db2.push(d)

    def _get_fb(self):
        with self._fb_lock:
            return self._fb

    def _wait_for_fb(self, timeout_s=3.0) -> bool:
        t0 = rospy.Time.now()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self._get_fb() is not None:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout_s:
                return False
            r.sleep()
        return False

    # ---------------- MoveIt helpers ----------------
    def go_joints(self, joints) -> bool:
        self.group.set_start_state_to_current_state()
        ok = self.group.go(list(joints), wait=True)
        self.group.stop()
        rospy.sleep(self.pause_after_move_s)
        return bool(ok)

    def go_home(self) -> bool:
        self.group.set_start_state_to_current_state()
        self.group.set_named_target("home")
        ok = self.group.go(wait=True)
        self.group.stop()
        rospy.sleep(self.pause_after_move_s)
        return bool(ok)

    # ---------------- Gripper helpers ----------------
    def _detect_pos_mode(self) -> str:
        if self._pos_mode is not None:
            return self._pos_mode

        if not self._wait_for_fb(timeout_s=3.0):
            self._pos_mode = "meters"
            rospy.logwarn("[handoff] No feedback yet; defaulting gripper position mode to METERS.")
            return self._pos_mode

        req = float(self._get_fb().requested_position)
        self._pos_mode = "meters" if req <= 0.2 else "raw255"
        rospy.loginfo(f"[handoff] Gripper position mode: {self._pos_mode} (requested_position={req})")

        # If we're in raw255 mode and taken_delta_pos is in meters-scale, bump it.
        if self._pos_mode == "raw255" and self.taken_delta_pos < 1.0:
            rospy.logwarn("[handoff] raw255 mode detected but taken_delta_pos looks small. "
                          "Consider setting _taken_delta_pos:=10 (typical).")
        return self._pos_mode

    def set_gripper(self, open_ratio: float, force: float):
        """open_ratio: 0.0 closed, 1.0 open"""
        mode = self._detect_pos_mode()
        r = clamp(open_ratio, 0.0, 1.0)

        goal = CommandRobotiqGripperGoal()
        if mode == "meters":
            goal.position = r * self.gripper_max_open_m
        else:
            goal.position = r * 255.0

        goal.speed = float(self.gripper_speed)
        goal.force = float(force)

        self.gripper.send_goal(goal)
        self.gripper.wait_for_result()

    # ---------------- Sensor waits & gating ----------------
    def wait_for_sensor1(self) -> bool:
        self.db1.reset()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.db1.fired(self.sensor1_threshold_cm):
                return True
            r.sleep()
        return False

    def arm_sensor2(self):
        self.sensor2_armed = True
        self.sensor2_arm_time = rospy.Time.now()
        self.db2.reset()

    def disarm_sensor2(self):
        self.sensor2_armed = False
        self.sensor2_arm_time = None
        self.db2.reset()

    def wait_for_sensor2_when_armed(self) -> bool:
        if not self.sensor2_armed:
            return False

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            # timeout protection
            if self.sensor2_timeout_s > 0.0 and self.sensor2_arm_time is not None:
                if (rospy.Time.now() - self.sensor2_arm_time).to_sec() > self.sensor2_timeout_s:
                    rospy.logwarn("[handoff] Sensor 2 timeout after Sensor 1. Disarming and re-waiting Sensor 1.")
                    self.disarm_sensor2()
                    return False

            if self.db2.fired(self.sensor2_threshold_cm):
                return True

            r.sleep()

        return False

    # ---------------- TAKEN detection: by position increase ----------------
    def wait_until_taken_by_position(self) -> bool:
        """
        Detect TAKEN by observing gripper position increase at handoff.
        Baseline is recorded at function start.
        TAKEN if position increases by >= taken_delta_pos for taken_stable_n samples.
        """
        if not self._wait_for_fb(timeout_s=3.0):
            rospy.logwarn("[handoff] No gripper feedback; cannot detect TAKEN.")
            return False

        fb0 = self._get_fb()
        if fb0 is None:
            return False

        baseline = float(fb0.position)
        stable = 0
        r = rospy.Rate(50)
        t0 = rospy.Time.now()

        rospy.loginfo(f"[handoff] TAKEN baseline position = {baseline:.6f}, delta threshold = {self.taken_delta_pos}")

        while not rospy.is_shutdown():
            if (rospy.Time.now() - t0).to_sec() > self.taken_timeout_s:
                return False

            fb = self._get_fb()
            if fb is None:
                r.sleep()
                continue

            pos = float(fb.position)
            if (pos - baseline) >= self.taken_delta_pos:
                stable += 1
            else:
                stable = 0

            if stable >= self.taken_stable_n:
                rospy.loginfo(f"[handoff] TAKEN detected: position increased to {pos:.6f}")
                return True

            r.sleep()

        return False

    # ---------------- Core sequence pieces ----------------
    def wait_pick_then_close_gated(self) -> bool:
        """
        Enforces:
          - Sensor 2 can only trigger AFTER Sensor 1 triggers
          - Robot closes gripper ONLY on Sensor 2 trigger
        """
        rospy.loginfo("[handoff] Waiting for Sensor 1 (go WAIT/PICK, arm Sensor 2)")
        if not self.wait_for_sensor1():
            return False

        rospy.loginfo("[handoff] Sensor 1 triggered -> moving to WAIT/PICK")
        if not self.go_joints(self.wait_pick_joints):
            rospy.logwarn("[handoff] Failed to reach WAIT/PICK")
            return False

        rospy.loginfo("[handoff] Arming Sensor 2")
        self.arm_sensor2()

        rospy.loginfo("[handoff] Waiting for Sensor 2 (armed) -> CLOSE gripper")
        if not self.wait_for_sensor2_when_armed():
            return False

        if self.close_delay_after_sensor2_s > 0.0:
            rospy.sleep(self.close_delay_after_sensor2_s)

        rospy.loginfo("[handoff] Sensor 2 triggered (armed) -> CLOSING gripper (ONLY close event)")
        self.set_gripper(open_ratio=0.0, force=self.force_close_on_sensor2)

        self.disarm_sensor2()
        return True

    def handoff_wait_taken_open_return(self) -> bool:
        """
        - Go to handoff pose
        - Optionally relax slightly OPEN (makes pulling easier; OPEN only)
        - Wait for TAKEN by position change
        - Open fully
        - Return to WAIT/PICK
        """
        rospy.loginfo("[handoff] Moving to HANDOFF pose")
        if not self.go_joints(self.handoff_joints):
            return False

        # Relax (OPEN only) for easy pulling
        if self.handoff_relax_open_ratio > 0.0:
            rospy.loginfo(f"[handoff] Relaxing gripper OPEN to ratio={self.handoff_relax_open_ratio:.2f} (OPEN only)")
            self.set_gripper(open_ratio=self.handoff_relax_open_ratio, force=self.force_open)

        rospy.loginfo("[handoff] Waiting for TAKEN (by gripper position increase)")
        taken = self.wait_until_taken_by_position()

        rospy.loginfo("[handoff] Opening gripper at handoff")
        self.set_gripper(open_ratio=1.0, force=self.force_open)

        rospy.loginfo("[handoff] Returning to WAIT/PICK")
        self.go_joints(self.wait_pick_joints)
        return taken

    def place_bundle_mid_place_open_home(self):
        rospy.loginfo("[handoff] Moving to MID bundle pose")
        self.go_joints(self.mid_bundle_joints)

        rospy.loginfo("[handoff] Moving to PLACE bundle pose")
        self.go_joints(self.place_bundle_joints)

        rospy.loginfo("[handoff] Opening gripper to release bundle")
        self.set_gripper(open_ratio=1.0, force=self.force_open)

        rospy.loginfo("[handoff] Returning HOME")
        self.go_home()

    # ---------------- Main loop ----------------
    def spin(self):
        rospy.loginfo("[handoff] Startup: HOME + open gripper")
        self.go_home()
        self.set_gripper(open_ratio=1.0, force=self.force_open)

        while not rospy.is_shutdown():
            rospy.loginfo("[handoff] ===== New cycle: 3 singles + bundle =====")

            # ---- Singles ----
            for i in range(self.num_boxes):
                rospy.loginfo(f"[handoff] --- Single box {i+1}/{self.num_boxes} ---")

                while not rospy.is_shutdown():
                    if self.wait_pick_then_close_gated():
                        break
                    rospy.sleep(0.2)

                if rospy.is_shutdown():
                    break

                taken = self.handoff_wait_taken_open_return()
                if not taken:
                    rospy.logwarn("[handoff] TAKEN not confirmed (timeout). Continuing anyway.")

            if rospy.is_shutdown():
                break

            # ---- Bundle ----
            rospy.loginfo("[handoff] --- Bundle phase ---")
            while not rospy.is_shutdown():
                if self.wait_pick_then_close_gated():
                    break
                rospy.sleep(0.2)

            if rospy.is_shutdown():
                break

            self.place_bundle_mid_place_open_home()
            rospy.sleep(self.idle_between_cycles_s)


def main():
    rospy.init_node("ultrasonic_handoff_bundle_sensor2_gated_taken_by_position", anonymous=False)
    UR3eHandoffBundle().spin()


if __name__ == "__main__":
    main()