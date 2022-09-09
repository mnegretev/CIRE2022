#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

from copy import deepcopy
import math
import sys

import geometry_msgs.msg
import moveit_commander
import rospy


class MoveItCartesianDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        arm = moveit_commander.MoveGroupCommander("arm")
        base = moveit_commander.MoveGroupCommander("base")
        head = moveit_commander.MoveGroupCommander("head")
        whole_body = moveit_commander.MoveGroupCommander("whole_body")
        scene = moveit_commander.PlanningSceneInterface()
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
        table_size = [0.01, 0.6, 0.6]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.53
        p.pose.position.y = 0.0
        p.pose.position.z = 0.6
        p.pose.orientation.w = 1.0
        scene.add_box("wall", p, table_size)
        rospy.sleep(1)

        # draw circle
        rospy.loginfo("step2: draw circle")
        waypoints = []
        p = geometry_msgs.msg.Pose()
        p.position.x = 0.4
        p.orientation.x = 0.707
        p.orientation.y = 0
        p.orientation.z = 0.707
        p.orientation.w = 0

        r = 0.2
        num_waypoints = 30
        for t in range(num_waypoints):
            th = math.pi * 2.0 / num_waypoints * t
            p.position.z = r * math.cos(th) + 0.6
            p.position.y = r * math.sin(th)
            waypoints.append(deepcopy(p))

        (plan, fraction) = whole_body.compute_cartesian_path(waypoints,
                                                             0.01,
                                                             0.0,
                                                             True)
        whole_body.execute(plan)
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItCartesianDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
