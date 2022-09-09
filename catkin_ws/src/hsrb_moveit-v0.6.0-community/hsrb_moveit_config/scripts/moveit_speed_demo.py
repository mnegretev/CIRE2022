#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import moveit_commander
import rospy


class MoveItSpeedDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)

        robot = moveit_commander.RobotCommander()
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

        # move to another state
        rospy.loginfo("step2: move")
        arm.set_joint_value_target({"arm_lift_joint": 0.4,
                                    "arm_flex_joint": -0.4,
                                    "arm_roll_joint": 0.0,
                                    "wrist_flex_joint": 0.4,
                                    "wrist_roll_joint": 1.57})
        traj = arm.plan()
        arm.execute(traj)
        rospy.logdebug("done")
        rospy.sleep(wait)

        # move_to_neutral
        rospy.loginfo("step3: move_to_neutral")
        arm.set_named_target("neutral")
        arm.go()
        rospy.sleep(wait)

        # move slowly
        rospy.loginfo("step4: move slowly")
        new_traj = arm.retime_trajectory(robot.get_current_state(),
                                         traj,
                                         0.2)
        arm.execute(new_traj)
        rospy.logdebug("done")
        rospy.sleep(wait)

        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItSpeedDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
