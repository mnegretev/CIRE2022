#!/usr/bin/env python3

import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import smach

import ros_numpy
from utils_evasion import *


########## Functions for takeshi states ##########


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

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado

        print('robot Estado S_0')
        # se toman las lecturas cuantizadas
        si,sd=get_lectura_cuant()
        
        if (si==0 and sd==0): 
            #####Accion
        	move_forward()
        	return 'outcome1'
            
        if (si==0 and sd==1): return 'outcome2'
        if (si==1 and sd==0): return 'outcome3'
        if (si==1 and sd==1): return 'outcome4'

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
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_2')
        #####Accion
        turn_left()
        return 'outcome1'



class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_3')
        #####Accion
        move_backward()
        return 'outcome1'



class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_4')
        #####Accion
        turn_right()
        return 'outcome1' 


class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_5')
        #####Accion
        move_backward()
        return 'outcome1' 



class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_6')
        #####Accion
        turn_left()
        return 'outcome1' 


class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_7')
        #####Accion
        turn_left()
        return 'outcome1' 


class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_8')
        #####Accion
        move_forward()
        return 'outcome1' 


class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_9')
        #####Accion
        move_forward()
        return 'outcome1' 


class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_10')
        #####Accion
        turn_right()
        return 'outcome1' 



class S11(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_11')
        #####Accion
        turn_right()
        return 'outcome1'                                  
        




def init(node_name):
    global laser, base_vel_pub
    rospy.init_node(node_name)
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
    laser = Laser()  


#Entry point
if __name__== '__main__':

    print("STATE MACHINE...")
    init("takeshi_smach")
    sm = smach.StateMachine(outcomes = ['END'])     #State machine, final state "END"
    sm.userdata.sm_counter = 0
    sm.userdata.clear = False


    with sm:
        #State machine for evasion
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_0', 'outcome2':'s_1','outcome3':'s_3','outcome4':'s_5'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_4','outcome2':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_0','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_6','outcome2':'END'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_7','outcome2':'END'})
        smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_8','outcome2':'END'})
        smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_9','outcome2':'END'})
        smach.StateMachine.add("s_9",   S9(),  transitions = {'outcome1':'s_10','outcome2':'END'})
        smach.StateMachine.add("s_10",  S10(), transitions = {'outcome1':'s_11','outcome2':'END'})
        smach.StateMachine.add("s_11",  S11(), transitions = {'outcome1':'s_0','outcome2':'END'})


outcome = sm.execute()


























