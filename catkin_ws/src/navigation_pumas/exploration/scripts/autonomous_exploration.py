#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import math
import tf
import os
import rospy
from navig_msgs.srv import GetInflatedMap, GetBoundaryPoints, GetGoalPoint
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool as Flag
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker



class Node:

    def __init__(self):
        self.init_node=rospy.init_node("autonomous_exploration")#We initialize the node
        #Constructing the publisher to pass the goal point to the move_pf server
        self.move_to_goal=rospy.Publisher('/navigation/move_base_simple/goal',Point,queue_size=10)
        #Constructing the publisher to pass the points for the visualization in rviz
        self.pub_vis =rospy.Publisher('/visualization_marker',Marker, queue_size=10)
        #Constructing the object Marker with the features of the visualizated points 
        self.markers=Marker(ns="points",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
        self.markers.header.stamp=rospy.Time()
        self.markers.header.frame_id="/map"
        self.markers.pose.orientation.w=1.0
        self.markers.pose.position.x=0 
        self.markers.pose.position.y=0
        self.markers.pose.position.z=0
        self.markers.scale.x=0.2
        self.markers.scale.y=0.2
        self.markers.scale.z=0.2
        self.markers.color.r=1.0#Setting the Color 
        self.markers.color.a=1.0
        #Attributes where the returning message of the services are saved
        self.data_mp=0
        self.data_im=0
        self.data_bp=0
        self.data_pr=0
        self.data_o=0
        self.data_v=0
        self.data_centroids=0
        self.inflated_cells=0.3#Attribute where the inflated ratio is defined
        #Attributes where the handler of the services are saved
	self.client_map=0
        self.client_pos_robot=0
        self.client_inflated_map=0
        self.client_boundary_points=0
        self.client_visualization=0
        self.client_centroids=0
        self.client_objetive=0
        self.cliente_mr=0
        self.last_objective=Point()#Instance of the object point to save the last objective selected
        #Definition of the initial conditions in a random value different of 0
        self.last_objective.x=100
        self.last_objective.y=100
        #Definition of all the attributes related with the position of the robot
        self.pos_a_robot=0
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.flag=True#Definition of the flag value to know when the goal point has been reached
        self.listener=tf.TransformListener()#Transformation to get the attributes related with the position of the robot in the getPosRobot method
        
    def visualization_points(self,points):

        self.markers.points=points
        
        self.pub_vis.publish(self.markers)   
        
        
    def getPosRobot(self):

        try:
            ([self.pos_x_robot, self.pos_y_robot, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            self.pos_a_robot = 2*math.atan2(rot[2], rot[3])
            if self.pos_a_robot > math.pi:
                self.pos_a_robot = self.pos_a_robot- 2*math.pi
            elif self.pos_a_robot<=-math.pi:
                self.pos_a_robot = self.pos_a_robot+ 2*math.pi

        except:
            pass

    def callback_move_robot_response(self,msg):
        self.flag=msg.data


    def autonomous_exploration(self):
        #Defining the Subscriber that listen the value of the Flag to changing it when the goal point is reached
        rospy.Subscriber('/move_base_simple/goal_response',Flag, self.callback_move_robot_response)

        if self.flag==True:#Just when the goal point is reached, we repeat all the process
            #os.system("clear")#Command to clean the terminal

            #-------------........--Map Client------------------------------------
            #print("Establishing the connection with Map Server")
            rospy.wait_for_service('/dynamic_map')#We are waiting to the connection with the server
            try:
                if self.client_map==0:
                    self.client_map=rospy.ServiceProxy('/dynamic_map',GetMap)#We create the handler of the Service

                self.data_mp=self.client_map()#We call the service
                
            
            except rospy.ServiceException as e:
                #print("The request for the map server failed: %s"%e)
		pass

            
            #print("Already we get the data related with the GetMap service\n")

            #----------------------------------------------------------------------
            

        
            #-----------------------Inflated Map Client--------------------------------
            #print("Establishing the connection with Inflated Map Server")
            rospy.wait_for_service('/navigation/mapping/get_inflated_map')#We are waiting to the connection with the server
            try:
                if self.client_inflated_map==0:
                    self.client_inflated_map=rospy.ServiceProxy('/navigation/mapping/get_inflated_map',GetInflatedMap)#We create the handler of the Service

                self.data_im=self.client_inflated_map(inflated_cells=self.inflated_cells, map=self.data_mp.map)#We call the service
                
            
            except rospy.ServiceException as e:
                #print("The request for the inflated map server failed: %s"%e)
		pass

            
            #print("Already we get the data related with the GetInflatedMap service\n")

        
            #--------------------------GetRobotPos------------------------------------
        
            self.getPosRobot()#We call the method getPosRobot 
            #print("Already we get the data related with the GetPosRobot service\n")
            #print("The position of the robot in [m] is: "+str([self.pos_x_robot,self.pos_y_robot])+"\n")
            #----------------------------------------------------------------------

     
            #-----------------------Centroids Client--------------------------------
            #print("Establishing the connection with Centroids Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points_clustered')#We are waiting to the connection with the server
            try:
                if self.client_centroids==0:
                    self.client_centroids=rospy.ServiceProxy('/navigation/mapping/get_boundary_points_clustered',GetBoundaryPoints)#We create the handler of the Service
                
                self.data_centroids=self.client_centroids(map=self.data_im.inflated_map)#We call the service
                
            
            except rospy.ServiceException as e:
                #print("The request for the centorids server failed: %s"%e)
		pass

            
            #print("Already we get {0} clusters from the boundary points\n".format(len(self.data_centroids.points))) 
            
            
            #self.visualization_points(self.data_centroids.points)
            
            #----------------Goal Point Client--------------------------------
            #print("Establishing the connection with Goal Point Server")
            rospy.wait_for_service('/navigation/mapping/get_goal_point')#We are waiting to the connection with the server
            try:
                if self.client_objetive==0:
                    self.client_objetive=rospy.ServiceProxy('/navigation/mapping/get_goal_point',GetGoalPoint)#We create the handler of the Service

                self.data_o=self.client_objetive(pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,pos_a_robot=self.pos_a_robot,points=self.data_centroids.points,last_objective=self.last_objective,method="angle_and_distance")#We call the service
                
            
            except rospy.ServiceException as e:
                #print("The request for the objective point server failed: %s"%e)
		pass

            
            #print("Already we get the objective: [{} , {}]".format(self.data_o.goal.x,self.data_o.goal.y)) 
            
            
            
            #----------------------Move Robot by Potential Fields--------------------------------
        
            self.flag=False#Change the value of the flag, because the goal point hasn't been reached
            self.move_to_goal.publish(self.data_o.goal)#We publish the goal point to move the robot by potential fields
            #print("\nEstablishing the connection with Potential Fields Node")
            #print("Wating to reach the goal")
            
    
            #------------------------------------------------------------------------------------
         
            #-----------------------Update the last pose----------------------------------------
            self.last_objective=self.data_o.goal#We update the last objective point   
            #------------------------------------------------------------------------------------
                
    

if __name__== "__main__":
    
    node=Node()
    loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            node.autonomous_exploration()
            loop.sleep()
        except:
            pass 
    
