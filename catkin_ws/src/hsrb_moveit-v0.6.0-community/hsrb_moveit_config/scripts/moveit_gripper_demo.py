#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation

import sys

import moveit_commander
import moveit_msgs.msg
import rospy
import trajectory_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf


class MoveItGripperDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo")


        #whole_body = moveit_commander.MoveGroupCommander("whole_body")
        arm = moveit_commander.MoveGroupCommander('arm')
        print(arm.get_current_joint_values())
        arm.set_named_target("neutral")
        arm.go()
        print(arm.get_current_joint_values())
        rospy.sleep(.5)

        #print(whole_body.get_current_joint_values())


        p = PoseStamped()
        p.header.frame_id = "/odom"
        p.pose.position.x = 0
        p.pose.position.y = 0
        #p.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, 0)   

            
        #whole_body.set_pose_target(p)
        #whole_body.go()
        #print(whole_body.get_current_joint_values())
        #whole_body.set_joint_value_target("world_joint",1,1,0)
        
        gripper = moveit_commander.MoveGroupCommander("gripper")



        
        #whole_body = moveit_commander.MoveGroupCommander("whole_body")
        #whole_body.set_workspace([-0.5, -1.0, 0.5, 1.0])
        #rospy.sleep(1.5)
        print(gripper.get_active_joints())

        rospy.sleep(.5)
        #arm = moveit_commander.MoveGroupCommander("arm")
      
        
        # move_to_neutral
        
        #rospy.loginfo("step1: move_to_neutral")
        #base.go()
        #arm.set_named_target("neutral")
        #arm.go()
        #head.set_named_target("neutral")
        #head.go()
        #rospy.logdebug("done")
        #rospy.sleep(wait)


        
        gripper.set_goal_joint_tolerance(0.05)

        # open
        gripper.set_joint_value_target("hand_motor_joint", 1.2)
        gripper.go()
        rospy.loginfo("step1: open")
        rospy.sleep(wait)

        # close
        gripper.set_joint_value_target("hand_motor_joint", 0.0)
        gripper.go()
        rospy.loginfo("step2: close")
        rospy.sleep(wait)

        # open again
        gripper.set_joint_value_target("hand_motor_joint", 1.2)
        gripper.go()
        rospy.loginfo("step3: open")
        rospy.sleep(wait)

        # grasp (no planning)
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0]
        p.effort = [-0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        gripper.execute(t)
        rospy.loginfo("step4: grasp")
        rospy.sleep(wait)

        
        # finalize
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItGripperDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
