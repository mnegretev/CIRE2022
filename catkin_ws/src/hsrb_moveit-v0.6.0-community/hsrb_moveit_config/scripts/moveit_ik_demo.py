#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import rospy


class MoveItIKDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_demo', anonymous=True)

        arm = moveit_commander.MoveGroupCommander('arm')
        head = moveit_commander.MoveGroupCommander('head')
        whole_body = moveit_commander.MoveGroupCommander('whole_body')
        whole_body_weighted \
            = moveit_commander.MoveGroupCommander('whole_body_weighted')
        whole_body_light \
            = moveit_commander.MoveGroupCommander('whole_body_light')

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move hand forward
        rospy.loginfo("step2: move hand forward")
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "hand_palm_link"
        p.pose.position.z = 0.4
        p.pose.orientation.w = 1
        whole_body.set_joint_value_target(p)
        whole_body.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move hand forward(whole_body_wighted)
        rospy.loginfo("step3: move hand forward (weighted)")
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        whole_body_weighted.set_joint_value_target(p)
        whole_body_weighted.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move hand forward(whole_body_light)
        rospy.loginfo("step4: move hand forward (light)")
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        whole_body_light.set_joint_value_target(p)
        whole_body_light.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItIKDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
