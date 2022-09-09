#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import rospy


class MoveAttachedObjectDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        arm = moveit_commander.MoveGroupCommander("arm")
        base = moveit_commander.MoveGroupCommander("base")
        head = moveit_commander.MoveGroupCommander("head")
        whole_body = moveit_commander.MoveGroupCommander("whole_body_weighted")
        scene = moveit_commander.PlanningSceneInterface()
        arm.allow_replanning(True)
        whole_body.allow_replanning(True)
        whole_body.set_workspace([-0.5, -0.5, 0.5, 0.5])
        rospy.sleep(1)

        # remove all objects
        end_effector_link = whole_body.get_end_effector_link()
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        base.go()
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # attach a stick to end_effector
        stick_size = [0.4, 0.02, 0.02]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = end_effector_link
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = 0.1 + stick_size[0] / 2
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0.707
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0.707
        scene.attach_box(end_effector_link, "stick", p, stick_size)

        # add table
        table_size = [0.4, 0.8, 0.01]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.83 + table_size[2] / 2.0
        p.pose.orientation.w = 1.0
        scene.add_box("table", p, table_size)
        rospy.sleep(1)

        # up arm
        rospy.loginfo("step2: move arm up")
        arm.set_joint_value_target({"arm_lift_joint": 0.3})
        arm.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # down arm
        rospy.loginfo("step3: move arm down")
        whole_body.set_joint_value_target({"arm_lift_joint": 0.0,
                                           "arm_flex_joint": -1.57})
        whole_body.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveAttachedObjectDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
