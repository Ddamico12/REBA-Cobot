#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def main():
    # initialize moveit_commander and rospy  
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('export_joint_states', anonymous=True)

    # replace "manipulator" with your planning group name (e.g. "gripper")
    group = moveit_commander.MoveGroupCommander("manipulator")

    # give MoveIt a moment to connect
    rospy.sleep(1.0)

    # fetch current joint values (radians)
    joints = group.get_current_joint_values()

    # print as [a,b,c,d,e,f,g]
    print([float(f"{j:.6f}") for j in joints])

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
