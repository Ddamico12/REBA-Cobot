#!/usr/bin/env python3
import rospy
import moveit_commander
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal
import subprocess
from moveit_msgs.msg import RobotTrajectory


class RobotController:
    def __init__(self, move_group_name="manipulator"):
        moveit_commander.roscpp_initialize([])
        # rospy.init_node("robot_controller", anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(move_group_name)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.home_joints  = rospy.get_param('~home_joints',
                                 [0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
        
        rospy.loginfo("Robot Controller initialized")
        
    def get_current_joints(self):
        """Get the current position based on six joint values

        Returns:
            List(float): a list of floating numbers that represents the six joint values
        """
        joints = self.group.get_current_joint_values()
        rospy.loginfo(f"Current joint values: {joints}")
        return joints
    
    def move_to_joints(self, joint_values: list):
        """Move to a specific position based on six joint values

        Args:
            joint_values (list): a list of joint values

        Returns:
            bool: if the operation is successful, then return true, else return false
        """
        if len(joint_values) != len(self.group.get_current_joint_values()):
            rospy.logerr(f"Expected {len(self.group.get_current_joint_values())} joint values, got {len(joint_values)}")
            return False
        self.group.set_joint_value_target(joint_values)
        self.group.set_max_velocity_scaling_factor(0.7)
        self.group.set_max_acceleration_scaling_factor(0.7)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return success
    
    def go_home(self):
        """Go back to home position

        Returns:
            bool: calls the move_to_joints() function, which returns a boolean value(True/False)
        """
        rospy.loginfo("Moving to home joint positions...")
        return self.move_to_joints(self.home_joints)
    
    def get_current_pose(self):
        """Get the current position based on pose position and orientation

        Returns:
            geometry_msgs.msg.Pose: returns a Pose type that's based on MoveIt, which contains position and orientation 
        """
        pose = self.group.get_current_pose().pose
        rospy.loginfo(f"Current pose:\n{pose}")
        return pose
    
    def move_to_pose(self, pose: PoseStamped):
        """Move to a specific position based on both position values and orientation values

        Args:
            pose (PoseStamped): a pose that is defined by position: x, y, z and orientation: x, y, z, w

        Returns:
            bool: if the operation is successful, then return true, else return false
        """
        rospy.loginfo("Moving to target pose...")
        self.group.set_pose_target(pose)
        self.group.set_max_velocity_scaling_factor(0.7)
        self.group.set_max_acceleration_scaling_factor(0.7)
        success = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return success
    
    def move_to_xyz(self, x: float, y: float, z: float, frame="base_link"):
        """Move to a specific position only based on position.x, position.y and position.z(with random orientation)

        Args:
            x (float): x position of the target pose
            y (float): y position of the target pose
            z (float): z position of the target pose
            frame (str, optional): the frame that x,y,z is relative to. Defaults to "base_link".

        Returns:
            bool: calls the move_to_pose() function, which returns a boolean value(True/False)
        """
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        self.group.set_max_velocity_scaling_factor(0.7)
        self.group.set_max_acceleration_scaling_factor(0.7)
        return self.move_to_pose(pose)
    
    # def transform_pose(self, pose_stamped, target_frame="base_link"):
    #     try:
    #         trans = self.tf_buffer.transform(pose_stamped, target_frame, rospy.Duration(1.0))
    #         return trans
    #     except Exception as e:
    #         rospy.logwarn(f"Transform failed: {e}")
    #         return None
    

    def cartesian_z_lift(self, dz: float, eef_step: float = 0.005, avoid_collisions: bool = True) -> bool:
        """
        Straight-line lift in base_link +Z using MoveIt Cartesian planning.
        Compatible with MoveIt Python wrapper that expects:
        compute_cartesian_path(waypoints, eef_step, avoid_collisions)
        """
        waypoints = []

        start = self.group.get_current_pose().pose
        w = start
        w.position.z += dz
        waypoints.append(w)

        plan, fraction = self.group.compute_cartesian_path(
            waypoints,
            eef_step,
            avoid_collisions
        )

        rospy.loginfo(f"Cartesian path fraction: {fraction:.3f}")
        if fraction < 0.95:
            rospy.logwarn("Cartesian path planning incomplete; not executing for safety.")
            return False

        self.group.execute(plan, wait=True)
        self.group.stop()
        return True


    
    def transform_pose(self, pose_stamped, target_frame="base_link"):
        try:
            # 1) lookup the newest transform available
            tf_stamped = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rospy.Time(0),          # “latest” snapshot
                rospy.Duration(1.0)     # timeout
            )
            # 2) apply it to your PoseStamped
            return do_transform_pose(pose_stamped, tf_stamped)
        
        except (tf2_ros.LookupException,
                 tf2_ros.ConnectivityException,
                 tf2_ros.ExtrapolationException) as e:
             rospy.logwarn(f"Transform failed: {e}")
             return None
        
    def control_gripper(self, frac: float, speed=0.1, force=10.0):
        """Controls the Robotiq gripper

        Args:
            frac (float): the scale of open and close of the gripper (close:0.0 ~ open: 1.0)
            speed (float, optional): controls the closing and opening speed. Defaults to 0.1.
            force (float, optional): controls how much force it use to close/open. Defaults to 5.0.

        Returns:
            dict: returns a dictionary of specific gripper data
        """
        client = actionlib.SimpleActionClient('/command_robotiq_action', CommandRobotiqGripperAction)
        rospy.loginfo("Waiting for Robotiq gripper action server...")
        client.wait_for_server()
        
        goal = CommandRobotiqGripperGoal()
        goal.position = frac * 255
        goal.speed = speed
        goal.force = force
        
        client.send_goal(goal=goal)
        client.wait_for_result()
        
        result = client.get_result()
        rospy.loginfo(f"Gripper {'opened' if open else 'closed'} with result: {result}")
        return result
    
    
    def force_callback(self, msg):
        global force_reading
        force_reading = msg.wrench.force
        
    def wait_for_force_trigger(self, threshold=0.5, axis='z', confirm_cycles=5):
        """
        Wait for force on given axis to exceed threshold for multiple cycles.
        Prevents false triggers due to sensor noise.
        """
        rospy.loginfo("Waiting for external force on tool to exceed %.2f N on axis %s...", threshold, axis)
        rate = rospy.Rate(50)
        trigger_count = 0

        while not rospy.is_shutdown():
            if force_reading:
                force_value = getattr(force_reading, axis)
                rospy.loginfo_throttle(1, f"Current force on {axis}: {force_value:.2f} N")

                if abs(force_value) > threshold:
                    trigger_count += 1
                else:
                    trigger_count = 0  # reset if it drops below threshold

                if trigger_count >= confirm_cycles:
                    rospy.loginfo("Force threshold exceeded consistently — opening gripper!")
                    break

            rate.sleep()


    def zero_force_sensor(self):
        try:
            rospy.loginfo("Calling zero_ftsensor service...")
            subprocess.run(['rosservice', 'call', '/ur_hardware_interface/zero_ftsensor'], check=True)
            rospy.loginfo("Force-torque sensor zeroed.")
        except subprocess.CalledProcessError as e:
            rospy.logwarn("Failed to zero force-torque sensor: %s", e)