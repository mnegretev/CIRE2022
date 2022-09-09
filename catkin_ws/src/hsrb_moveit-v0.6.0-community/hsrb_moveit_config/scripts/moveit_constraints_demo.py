#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy


class MoveItConstraintsDemo(object):
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

        # add table
        table_size = [0.4, 0.8, 0.01]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.5
        p.pose.position.y = 0.0
        p.pose.position.z = 0.8 + table_size[2] / 2.0
        p.pose.orientation.w = 1.0
        scene.add_box("table", p, table_size)
        rospy.sleep(1)

        # create constraints
        constraints = moveit_msgs.msg.Constraints()
        constraints.name = "keep horizontal"
        orientation_constraint = moveit_msgs.msg.OrientationConstraint()
        orientation_constraint.header.frame_id = "odom"
        orientation_constraint.link_name = whole_body.get_end_effector_link()
        orientation_constraint.orientation.x = 0.707
        orientation_constraint.orientation.y = 0
        orientation_constraint.orientation.z = 0.707
        orientation_constraint.orientation.w = 0
        orientation_constraint.absolute_x_axis_tolerance = 3.14
        orientation_constraint.absolute_y_axis_tolerance = 0.001
        orientation_constraint.absolute_z_axis_tolerance = 0.001
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)

        whole_body.set_path_constraints(constraints)

        # move end effector
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "odom"
        p.pose.position.x = 0.4
        p.pose.position.y = 0.0
        p.pose.position.z = 1.0
        p.pose.orientation.x = 0.707
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0.707
        p.pose.orientation.w = 0

        rospy.loginfo("step2: move end effector with constraints")
        whole_body.set_joint_value_target(p)
        whole_body.go()
        rospy.logdebug("done")
        rospy.sleep(wait)

        whole_body.clear_path_constraints()

        # finalize
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItConstraintsDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
