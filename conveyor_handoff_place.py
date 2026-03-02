#!/usr/bin/env python3
"""
conveyor_handoff_place.py

Repeating conveyor cycle:
  Phase 1 — Pick N small boxes one at a time and hand each to a human
  Phase 2 — Pick 1 large box and place it on a shelf
  Repeat

Waypoints (taught joints):
  - wait_pick_joints
  - pre_handoff_joints  (NEW: between wait_pick and handoff)
  - handoff_joints
  - mid_joints
  - place_joints
  - post_place_joints   (NEW: between place and home)

Sensor gating:
  - Sensor 1 tells the robot to move to WAIT/PICK position
  - Sensor 2 is armed only after Sensor 1 fires (with timeout protection)
  - Gripper CLOSE happens only when Sensor 2 triggers

"Taken" detection at handoff is a simple sleep timer for now,
isolated in wait_until_taken() so it can be replaced later.

RUN
---
chmod +x conveyor_handoff_place.py
source ~/main_workspace/catkin_ws/devel/setup.bash
rosparam load /path/to/conveyor_joints.yaml
rosrun ur3e_moveit_config conveyor_handoff_place.py
"""

from collections import deque
import rospy
from std_msgs.msg import Float32
import moveit_commander
import actionlib
from robotiq_2f_gripper_msgs.msg import (
    CommandRobotiqGripperAction,
    CommandRobotiqGripperGoal,
)


# ------------------------------------------------------------------ #
#  Debounce helper                                                     #
# ------------------------------------------------------------------ #

class Debounce:
    """Triggers when the last N samples are all below a threshold."""

    def __init__(self, n_required: int):
        self.buf = deque(maxlen=max(1, int(n_required)))

    def push(self, v: float):
        self.buf.append(float(v))

    def fired(self, threshold: float) -> bool:
        return len(self.buf) == self.buf.maxlen and all(x < threshold for x in self.buf)

    def reset(self):
        self.buf.clear()


# ------------------------------------------------------------------ #
#  Main node                                                           #
# ------------------------------------------------------------------ #

class ConveyorHandoffPlace:
    def __init__(self):
        # ---------- Sensor parameters ----------
        self.start_threshold = float(rospy.get_param("~start_threshold_cm", 30.0))
        self.middle_threshold = float(rospy.get_param("~middle_threshold_cm", 30.0))
        self.invalid_below = float(rospy.get_param("~invalid_below_cm", 1.0))
        self.debounce_n = int(rospy.get_param("~debounce_n", 2))
        self.sensor2_timeout_s = float(rospy.get_param("~sensor2_timeout_s", 5.0))
        self.grab_delay_s = float(rospy.get_param("~grab_delay_s", 0.5))

        # ---------- Taken placeholder ----------
        self.taken_wait_s = float(rospy.get_param("~taken_wait_s", 3.0))

        # ---------- Cycle ----------
        self.num_small_boxes = int(rospy.get_param("~num_small_boxes", 3))

        # ---------- Motion scaling ----------
        self.velocity_scale = float(rospy.get_param("~velocity_scale", 0.3))
        self.accel_scale = float(rospy.get_param("~accel_scale", 0.3))

        # ---------- Gripper ----------
        self.gripper_speed = float(rospy.get_param("~gripper_speed", 255.0))
        self.gripper_force_close = float(rospy.get_param("~gripper_force_close", 75.0))
        self.gripper_force_open = float(rospy.get_param("~gripper_force_open", 50.0))

        # ---------- Joint targets (set via rosparam/YAML) ----------
        self.wait_pick_joints = rospy.get_param("~wait_pick_joints", [0, -1.57, 0, -1.57, 0, 0])
        self.pre_handoff_joints = rospy.get_param("~pre_handoff_joints", self.wait_pick_joints)  # NEW
        self.handoff_joints = rospy.get_param("~handoff_joints", [0, -1.57, 0, -1.57, 0, 0])

        self.mid_joints = rospy.get_param("~mid_joints", [0, -1.57, 0, -1.57, 0, 0])
        self.place_joints = rospy.get_param("~place_joints", [0, -1.57, 0, -1.57, 0, 0])
        self.post_place_joints = rospy.get_param("~post_place_joints", self.place_joints)  # NEW

        # ---------- Debouncers ----------
        self.db1 = Debounce(self.debounce_n)
        self.db2 = Debounce(self.debounce_n)

        # ---------- Sensor 2 gating state ----------
        self.sensor2_armed = False
        self.sensor2_arm_time = None

        # ---------- MoveIt ----------
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander("manipulator")
        self.group.set_max_velocity_scaling_factor(self.velocity_scale)
        self.group.set_max_acceleration_scaling_factor(self.accel_scale)

        # ---------- Gripper action client ----------
        self.gripper_ac = actionlib.SimpleActionClient(
            "/command_robotiq_action", CommandRobotiqGripperAction
        )
        rospy.loginfo("[cycle] Waiting for gripper action server...")
        self.gripper_ac.wait_for_server()
        rospy.loginfo("[cycle] Gripper action server connected.")

        # ---------- Ultrasonic subscribers ----------
        rospy.Subscriber("/ultrasonic_distance_start", Float32, self._cb_sensor1, queue_size=50)
        rospy.Subscriber("/ultrasonic_distance_middle", Float32, self._cb_sensor2, queue_size=50)

        rospy.loginfo("[cycle] Node initialized.")

    # ----------------------------------------------------------------
    #  Sensor callbacks
    # ----------------------------------------------------------------
    def _cb_sensor1(self, msg):
        d = float(msg.data)
        if d > self.invalid_below:
            self.db1.push(d)

    def _cb_sensor2(self, msg):
        d = float(msg.data)
        if d > self.invalid_below:
            self.db2.push(d)

    # ----------------------------------------------------------------
    #  MoveIt helpers
    # ----------------------------------------------------------------
    def go_joints(self, joints):
        self.group.set_start_state_to_current_state()
        ok = self.group.go(list(joints), wait=True)
        self.group.stop()
        rospy.sleep(0.05)  # slightly snappier than 0.15
        return bool(ok)

    def go_home(self):
        self.group.set_start_state_to_current_state()
        self.group.set_named_target("home")
        ok = self.group.go(wait=True)
        self.group.stop()
        rospy.sleep(0.05)
        return bool(ok)

    # ----------------------------------------------------------------
    #  Gripper helper
    # ----------------------------------------------------------------
    def set_gripper(self, open_ratio, force=None):
        """open_ratio: 0.0 = closed, 1.0 = fully open."""
        if force is None:
            force = self.gripper_force_open if open_ratio > 0.5 else self.gripper_force_close

        r = max(0.0, min(1.0, float(open_ratio)))
        goal = CommandRobotiqGripperGoal()
        goal.position = r * 255.0
        goal.speed = self.gripper_speed
        goal.force = float(force)
        self.gripper_ac.send_goal(goal)
        self.gripper_ac.wait_for_result()

    # ----------------------------------------------------------------
    #  Sensor waits & gating
    # ----------------------------------------------------------------
    def wait_for_sensor1(self):
        """Block until Sensor 1 fires (debounced). Returns True, or False on shutdown."""
        self.db1.reset()
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.db1.fired(self.start_threshold):
                return True
            r.sleep()
        return False

    def wait_for_sensor2(self):
        """Block until Sensor 2 fires (debounced) with timeout.

        Returns True on trigger, False on timeout or shutdown.
        """
        self.sensor2_armed = True
        self.sensor2_arm_time = rospy.Time.now()
        self.db2.reset()

        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.sensor2_timeout_s > 0.0 and self.sensor2_arm_time is not None:
                elapsed = (rospy.Time.now() - self.sensor2_arm_time).to_sec()
                if elapsed > self.sensor2_timeout_s:
                    rospy.logwarn("[cycle] Sensor 2 timed out. Disarming.")
                    self.sensor2_armed = False
                    self.sensor2_arm_time = None
                    self.db2.reset()
                    return False

            if self.db2.fired(self.middle_threshold):
                self.sensor2_armed = False
                self.sensor2_arm_time = None
                return True

            r.sleep()

        return False

    # ----------------------------------------------------------------
    #  Taken placeholder
    # ----------------------------------------------------------------
    def wait_until_taken(self):
        rospy.loginfo("[cycle] Waiting %.1f s for human to take box...", self.taken_wait_s)
        rospy.sleep(self.taken_wait_s)

    # ----------------------------------------------------------------
    #  Main loop
    # ----------------------------------------------------------------
    def spin(self):
        # -- Startup --
        rospy.loginfo("[cycle] Startup: moving HOME and opening gripper")
        self.go_home()
        self.set_gripper(1.0, force=self.gripper_force_open)

        while not rospy.is_shutdown():
            rospy.loginfo("[cycle] ===== New cycle: %d handoffs + 1 placement =====", self.num_small_boxes)

            # ============================================================
            #  PHASE 1: Small boxes — pick and hand off (x num_small_boxes)
            # ============================================================
            box_count = 0
            while box_count < self.num_small_boxes and not rospy.is_shutdown():
                rospy.loginfo("[cycle] --- Small box %d/%d ---", box_count + 1, self.num_small_boxes)

                # Sensor 1 → move to wait/pick
                rospy.loginfo("[cycle] Waiting for Sensor 1...")
                if not self.wait_for_sensor1():
                    break
                rospy.loginfo("[cycle] Sensor 1 triggered -> moving to WAIT/PICK")
                self.go_joints(self.wait_pick_joints)

                # Sensor 2 (gated) → close gripper
                rospy.loginfo("[cycle] Waiting for Sensor 2 (armed)...")
                if not self.wait_for_sensor2():
                    rospy.logwarn("[cycle] Sensor 2 timed out, re-waiting Sensor 1 for this box")
                    continue

                if self.grab_delay_s > 0:
                    rospy.sleep(self.grab_delay_s)

                rospy.loginfo("[cycle] Sensor 2 triggered -> closing gripper")
                self.set_gripper(0.0, force=self.gripper_force_close)

                # Move to handoff via NEW mid waypoint
                rospy.loginfo("[cycle] Moving to PRE-HANDOFF (mid waypoint)")
                self.go_joints(self.pre_handoff_joints)

                rospy.loginfo("[cycle] Moving to HANDOFF position")
                self.go_joints(self.handoff_joints)

                # Wait for human to take
                self.wait_until_taken()

                # Open gripper
                rospy.loginfo("[cycle] Opening gripper")
                self.set_gripper(1.0, force=self.gripper_force_open)

                box_count += 1

                # Return to wait/pick for next box (or skip if last)
                if box_count < self.num_small_boxes:
                    rospy.loginfo("[cycle] Returning to WAIT/PICK for next box")
                    self.go_joints(self.wait_pick_joints)

            if rospy.is_shutdown():
                break

            # Return HOME after all handoffs
            rospy.loginfo("[cycle] All handoffs done -> returning HOME")
            self.go_home()

            # ============================================================
            #  PHASE 2: Large box — pick and place (x1)
            # ============================================================
            rospy.loginfo("[cycle] --- Large box (place) ---")

            # Sensor 1 → move to wait/pick
            rospy.loginfo("[cycle] Waiting for Sensor 1...")
            if not self.wait_for_sensor1():
                break
            rospy.loginfo("[cycle] Sensor 1 triggered -> moving to WAIT/PICK")
            self.go_joints(self.wait_pick_joints)

            # Sensor 2 (gated) → close gripper (with retry)
            rospy.loginfo("[cycle] Waiting for Sensor 2 (armed)...")
            while not rospy.is_shutdown():
                if self.wait_for_sensor2():
                    break
                rospy.logwarn("[cycle] Sensor 2 timed out, re-waiting Sensor 1...")
                if not self.wait_for_sensor1():
                    break
                self.go_joints(self.wait_pick_joints)

            if rospy.is_shutdown():
                break

            if self.grab_delay_s > 0:
                rospy.sleep(self.grab_delay_s)

            rospy.loginfo("[cycle] Sensor 2 triggered -> closing gripper")
            self.set_gripper(0.0, force=self.gripper_force_close)

            # Move to mid, then place
            rospy.loginfo("[cycle] Moving to MID position")
            self.go_joints(self.mid_joints)

            rospy.loginfo("[cycle] Moving to PLACE position")
            self.go_joints(self.place_joints)

            # Release
            rospy.loginfo("[cycle] Opening gripper to release")
            self.set_gripper(1.0, force=self.gripper_force_open)

            # NEW: retreat via post-place waypoint before HOME
            rospy.loginfo("[cycle] Moving to POST-PLACE (mid waypoint)")
            self.go_joints(self.post_place_joints)

            rospy.loginfo("[cycle] Returning HOME")
            self.go_home()

            rospy.loginfo("[cycle] ===== Cycle complete =====")


def main():
    rospy.init_node("conveyor_handoff_place", anonymous=False)
    ConveyorHandoffPlace().spin()


if __name__ == "__main__":
    main()