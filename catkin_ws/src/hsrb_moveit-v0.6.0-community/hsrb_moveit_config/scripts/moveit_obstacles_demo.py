#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy


class MoveItObstaclesDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        arm = moveit_commander.MoveGroupCommander("arm")
        base = moveit_commander.MoveGroupCommander("base")
        head = moveit_commander.MoveGroupCommander("head")
        whole_body = moveit_commander.MoveGroupCommander("whole_body")
        scene = moveit_commander.PlanningSceneInterface()
        whole_body.set_workspace([-0.5, -1.0, 0.5, 1.0])
        self.scene_pub = rospy.Publisher('planning_scene',
                                         moveit_msgs.msg.PlanningScene,
                                         queue_size=5)
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

        # add wall
        wall_size = [0.4, 0.01, 1.0]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.4
        p.pose.position.y = -0.1
        p.pose.position.z = 0.6
        p.pose.orientation.w = 1.0
        scene.add_box("wall", p, wall_size)
        rospy.sleep(1)

        # set colors
        p = moveit_msgs.msg.PlanningScene()
        p.is_diff = True
        color = moveit_msgs.msg.ObjectColor()
        color.id = "wall"
        color.color.r = 0.6
        color.color.a = 0.9
        p.object_colors.append(color)
        self.scene_pub.publish(p)
        rospy.sleep(1)

        # move end effector
        rospy.loginfo("step2: move end effector")
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.2
        p.pose.position.y = -0.4
        p.pose.position.z = 1.0
        p.pose.orientation.x = 0.707
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0.707
        p.pose.orientation.w = 0
        whole_body.set_joint_value_target(p)
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
    MoveItObstaclesDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
