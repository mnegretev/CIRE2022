#!/usr/bin/env python
# Moveit chillvamp@hotmail.com
# OSCII CODE
# Copyright (C) 2017 Toyota Motor Corporation

import sys
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import tf
import ros_numpy
import numpy as np

## TOYOTA TABLETOP SEGMENTATOR
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest


##CV2 
from cv_bridge import CvBridge, CvBridgeError

import cv2

## TF 2's ( not tensorflow2 but TF 2)
import tf2_ros






class MoveItObstaclesDemo(object):
    def __init__(self, wait=0.0):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_demo", anonymous=True)
        service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
        service_client.wait_for_service(timeout=1.0)
        req = TabletopSegmentationRequest()
        
        tf_broadcaster =tf2_ros.StaticTransformBroadcaster() # tf2_ros.TransformBroadcaster()
        tfBuffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tfBuffer)




        bridge = CvBridge()



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


        ### DETECT PLANES
        
        req.crop_enabled = True
        req.crop_x_max = 1.0     # X coordinate maximum value in the area [m]
        req.crop_x_min = -1.0    # X coordinate minimum value in the area [m]
        req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
        req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
        req.crop_z_max = 1.8     # Z coordinate maximum value in the area [m]
        req.crop_z_min = 0.8     # Z coordinate minimum value in the area [m]
        req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
        req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
        req.remove_bg = False    # remove the background of the segment image



        res = service_client(req)
        #### READ SERVICE  RESPONSE 

        objs_depth_centroids=[]
        for i in range (len(res.segmented_objects_array.table_objects_array )):
            print ( 'Plane',i,'has', len(res.segmented_objects_array.table_objects_array[i].depth_image_array), 'objects')
            for j in range (len(res.segmented_objects_array.table_objects_array[i].points_array)):
                cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[0] )
                cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[0] )
                pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[j])
                points=np.zeros((pc.shape[0],3))
                points[:,0]=pc['x']
                points[:,1]=pc['y']
                points[:,2]=pc['z']
                objs_depth_centroids.append(np.mean(points,axis=0))
                print(i,j)
                
                

        for i in range (len(res.table_array.tables)):   
            print (res.table_array.tables[i].pose)

        rospy.loginfo('Number of detected objects={0}'.format(
            len(   objs_depth_centroids)))
        rospy.loginfo('Number of detected planes={0}'.format(
            len(res.table_array.tables)))

        ###PUBLISH TF
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        for ind, xyz in enumerate(objs_depth_centroids):
            
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obj"+str(ind), "head_rgbd_sensor_link")
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.child_frame_id = "TF2"+(str)(ind)
            static_transformStamped.header.frame_id = "head_rgbd_sensor_link"
            static_transformStamped.child_frame_id = "TF2"+(str)(ind)
            static_transformStamped.transform.translation.x = float(xyz[0])
            static_transformStamped.transform.translation.z = float(xyz[2])
            static_transformStamped.transform.translation.y = float(xyz[1])
            quat = tf.transformations.quaternion_from_euler(0,0,0)
            static_transformStamped.transform.rotation.x = quat[0]
            static_transformStamped.transform.rotation.y = quat[1]
            static_transformStamped.transform.rotation.z = quat[2]
            static_transformStamped.transform.rotation.w = quat[3]
        
            tf_broadcaster.sendTransform(static_transformStamped)

        
      

        # move_to_neutral
        rospy.loginfo("step1: move_to_neutral")
        #base.go()
        arm.set_named_target("neutral")
        arm.go()
        head.set_named_target("neutral")
        head.go()
        rospy.logdebug("done")
        rospy.sleep(wait)




        ## ADD SURFACE
        wall_size = [0.4, 0.01, 0.5]
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = "head_rgbd_sensor_link"
        p.pose.position.x = -0.202111288905
        p.pose.position.y = .3
        p.pose.position.z = 1


        quat = tf.transformations.quaternion_from_euler(1.57,0,0)
        

        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]
        scene.add_box("wall", p, wall_size)
        rospy.sleep(1)

        # set colors
        p = moveit_msgs.msg.PlanningScene()
        p.is_diff = True
        color = moveit_msgs.msg.ObjectColor()
        color.id = "wall"
        color.color.g = 0.6
        color.color.a = 0.9
        p.object_colors.append(color)
        self.scene_pub.publish(p)
        rospy.sleep(1)








        # finalize
        scene.remove_attached_object(end_effector_link)
        scene.remove_world_object()
        rospy.sleep(1)
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItObstaclesDemo(float(sys.argv[1]) if len(sys.argv) > 1 else 0.0)
