#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist , PoseStamped
import smach

import ros_numpy
from utils_evasion import *
import tf2_ros


########## Functions for takeshi states ##########


def get_coords ():
    for i in range(10):   ###TF might be late, try 10 times
        try:
            trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            return trans
        except:
            print ('waiting for tf')
            trans=0
            





def move_base_vel(vx, vy, vw):
    twist = Twist()
    twist.linear.x = vx
    twist.linear.y = vy
    twist.angular.z = vw 
    base_vel_pub.publish(twist)

def move_base(x,y,yaw,timeout=5):
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) 


def move_forward():
    move_base(0.15,0,0,2.5)
def move_backward():
    move_base(-0.15,0,0,1.5)
def turn_left():
    move_base(0.0,0,0.12*np.pi,2)
def turn_right():
    move_base(0.0,0,-0.12*np.pi,2)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) #remove infinito

        right_scan=lectura[:300]
        left_scan=lectura[300:]
        ront_scan=lectura[300:360]

        sd,si=0,0
        if np.mean(left_scan)< 3: si=1
        if np.mean(right_scan)< 3: sd=1

    except:
        sd,si=0,0    

    return si,sd




##### Define state INITIAL #####

class Inicio (smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succ','fail']) #shor for success
        
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('inicializando')
        ########
        
        rospy.sleep(1)#### dar tiempo al arbol de tfs de publicarse
        
       
        ##### agregue código para leer la meta en el tópico adecuado
        ###meta_leida=
        
        ############


        ######################################################################
        #meta_leida = PoseStamped() 
         ##### EJEMPLO los equipos deben leer la meta del topico, comentar esta linea
        ####################################################################
        punto_inicial = get_coords()
        print ( 'tiempo = '+ str(punto_inicial.header.stamp.to_sec()) , punto_inicial.transform )
        #print ('meta leida', meta_leida)
        print('arrancando')
        return 'succ'

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('robot Estado S_1')
        #####Accion
        move_backward()
        return 'outcome1'


class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
      
        


    def execute(self,userdata):
        print('robot Estado S_2')
        #####Accion
        punto_final=get_coords()
        print ( 'tiempo = '+ str(punto_final.header.stamp.to_sec()) , punto_final.transform )


        return 'outcome1'







def init(node_name):
    global laser, base_vel_pub
    rospy.init_node(node_name)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    laser = Laser()  


#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("smach_node")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    with sm:
        #State machine for evasion
        smach.StateMachine.add("INICIO",   Inicio(),  transitions = {'fail':'END', 'succ':'s_1'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'END','outcome2':'END'})
        


outcome = sm.execute()


























