# -*- coding: utf-8 -*-
import math
import numpy as np
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import ros_numpy


from sensor_msgs.msg import LaserScan

class Laser():
    u"""Class that handles laser information"""

    def __init__(self):
        # Register the _laser_cb method as a callback to the laser scan topic events
        self._laser_sub = rospy.Subscriber ('/hsrb/base_scan',
                                           LaserScan, self._laser_cb)
        self._scan_data = None

    def _laser_cb (self, msg):
        # Laser scan callback function
        self._scan_data = msg

    def get_data(self):
        u"""Function to get the laser value"""
        return self._scan_data