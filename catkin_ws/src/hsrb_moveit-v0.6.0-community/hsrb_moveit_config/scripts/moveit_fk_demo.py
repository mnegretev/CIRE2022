#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import moveit_commander
import rospy


class MoveItFKDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        arm = moveit_commander.MoveGroupCommander("arm")
        head = moveit_commander.MoveGroupCommander("head")

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move_to_go
        rospy.loginfo("step2: move_to_go")
        arm.set_named_target("go")
        arm.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # print joint_names
        rospy.loginfo("step3: get joint names: " +
                      str(arm.get_active_joints()))
        rospy.sleep(wait)

        # arm_lift_joint
        rospy.loginfo("step4: set arm_lift_joint 0.2")
        arm.set_named_target("neutral")
        arm.go()
        arm.set_joint_value_target("arm_lift_joint", 0.2)
        arm.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move head
        rospy.loginfo("step5: move head")
        head.set_joint_value_target({"head_pan_joint": 0.4,
                                     "head_tilt_joint": -0.2})
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItFKDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
